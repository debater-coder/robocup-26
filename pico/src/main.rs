//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates the possibility to send log::info/warn/error/debug! to USB serial port.

#![no_std]
#![no_main]

use core::f32::consts::TAU;
use core::iter::zip;

use crate::motor::MotorFeedback;
use cobs::{CobsDecoder, CobsEncoder};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::USB;
use embassy_rp::pwm::Pwm;
use embassy_rp::usb::{Driver, Instance, InterruptHandler};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use log::{info, warn};

use {defmt_rtt as _, panic_probe as _};

mod motor;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

static CONTROL_SIGNAL: Signal<CriticalSectionRawMutex, i32> = Signal::new();

static ODOM_SIGNAL_FL: Signal<CriticalSectionRawMutex, u32> = Signal::new();
static ODOM_SIGNAL_FR: Signal<CriticalSectionRawMutex, u32> = Signal::new();
static ODOM_SIGNAL_RL: Signal<CriticalSectionRawMutex, u32> = Signal::new();
static ODOM_SIGNAL_RR: Signal<CriticalSectionRawMutex, u32> = Signal::new();

const PULSES_PER_MM: f32 = 2520.0 / (48.0 * TAU);

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    let period: Duration = Duration::from_secs(2);

    loop {
        info!("led on!");
        led.set_high();
        Timer::after(period / 2).await;

        info!("led off!");
        led.set_low();
        Timer::after(period / 2).await;
    }
}

#[embassy_executor::task]
async fn odom_task(
    mut odom_pin: Input<'static>,
    signal: &'static Signal<CriticalSectionRawMutex, u32>,
) {
    let mut odom: u32 = 0;

    loop {
        odom_pin.wait_for_any_edge().await;
        odom += 1;
        signal.signal(odom);
    }
}

#[embassy_executor::task]
async fn kinematics_task(
    fl: MotorFeedback,
    rl: MotorFeedback,
    rr: MotorFeedback,
    fr: MotorFeedback,
) {
    let mut ticker = Ticker::every(Duration::from_hz(20));
    let mut motors = [fl, rl, rr, fr];
    loop {
        for (motor, signal) in zip(
            &mut motors,
            [
                &ODOM_SIGNAL_FL,
                &ODOM_SIGNAL_RL,
                &ODOM_SIGNAL_RR,
                &ODOM_SIGNAL_FR,
            ],
        ) {
            if let Some(speed) = CONTROL_SIGNAL.try_take() {
                motor.target = speed;
            }

            motor.update(signal.try_take().unwrap_or(0));
        }

        ticker.next().await;
    }
}

/// This will allow resets in case of panic (but not any other type of hand)
#[embassy_executor::task]
async fn feed_watchdog(mut watchdog: Watchdog) {
    loop {
        watchdog.feed();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let mut watchdog = Watchdog::new(p.WATCHDOG);
    watchdog.start(Duration::from_millis(500));
    spawner.spawn(feed_watchdog(watchdog)).unwrap();

    let led = Output::new(p.PIN_25, Level::Low);
    spawner.spawn(led_task(led)).unwrap();

    spawner
        .spawn(odom_task(Input::new(p.PIN_11, Pull::Up), &ODOM_SIGNAL_FL))
        .unwrap();
    spawner
        .spawn(odom_task(Input::new(p.PIN_13, Pull::Up), &ODOM_SIGNAL_RL))
        .unwrap();
    spawner
        .spawn(odom_task(Input::new(p.PIN_21, Pull::Up), &ODOM_SIGNAL_RR))
        .unwrap();
    spawner
        .spawn(odom_task(Input::new(p.PIN_19, Pull::Up), &ODOM_SIGNAL_FR))
        .unwrap();

    spawner
        .spawn(kinematics_task(
            MotorFeedback::new(
                Output::new(p.PIN_2, Level::Low),
                Pwm::new_output_b(p.PWM_SLICE1, p.PIN_3, Default::default())
                    .split()
                    .1
                    .unwrap(),
            ),
            MotorFeedback::new(
                Output::new(p.PIN_4, Level::Low),
                Pwm::new_output_b(p.PWM_SLICE2, p.PIN_5, Default::default())
                    .split()
                    .1
                    .unwrap(),
            ),
            MotorFeedback::new(
                Output::new(p.PIN_6, Level::Low),
                Pwm::new_output_b(p.PWM_SLICE3, p.PIN_7, Default::default())
                    .split()
                    .1
                    .unwrap(),
            ),
            MotorFeedback::new(
                Output::new(p.PIN_8, Level::Low),
                Pwm::new_output_b(p.PWM_SLICE4, p.PIN_9, Default::default())
                    .split()
                    .1
                    .unwrap(),
            ),
        ))
        .unwrap();

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("High Scorers");
    config.product = Some("USB Motor Controller");
    config.serial_number = Some("RC-67");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();
    let mut logger_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Create a class for the logger
    let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);

    // Creates the logger and returns the logger future
    // Note: You'll need to use log::info! afterwards instead of info! for this to work (this also applies to all the other log::* macros)
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let command_fut = async {
        loop {
            class.wait_connection().await;
            log::info!("Connected");
            let _ = handle_commands(&mut class).await;
            log::info!("Disconnected");
        }
    };

    join(usb_fut, join(command_fut, log_fut)).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn handle_commands<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    let mut dest = [0; 1024];
    let mut decoder = CobsDecoder::new(&mut dest);
    let mut odom = 0;

    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];

        for byte in data {
            match decoder.feed(*byte) {
                Err(e) => {
                    warn!("Error parsing packet: {:?}", e);
                }
                Ok(None) => {}
                Ok(Some(n)) => {
                    if n != 4 {
                        warn!("Invalid packet size");
                    } else {
                        let mut control_dst = [0u8; 4];
                        control_dst.copy_from_slice(&decoder.dest()[..4]);
                        let control = i32::from_be_bytes(control_dst);

                        info!("Received control: {}", control);

                        CONTROL_SIGNAL.signal((control as f32 * PULSES_PER_MM) as i32);

                        if let Some(odom_val) = ODOM_SIGNAL_FL.try_take() {
                            odom = odom_val;
                        }

                        // Return period in milliseconds
                        let mut out_buf = [0u8; 62];
                        let mut encoder = CobsEncoder::new(&mut out_buf);
                        let Ok(_) =
                            encoder.push(&((odom as f32 / PULSES_PER_MM) as i32).to_be_bytes())
                        else {
                            warn!("Error encoding data!");
                            continue;
                        };
                        encoder.finalize();
                        class.write_packet(&[0]).await?;
                        class.write_packet(&out_buf).await?;
                        class.write_packet(&[0]).await?;
                    }
                }
            }
        }

        info!("data: {:?}", data);
    }
}
