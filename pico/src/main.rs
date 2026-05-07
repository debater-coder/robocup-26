#![no_std]
#![no_main]

use core::f32::consts::PI;
use core::iter::zip;

use crate::kinematics::ChassisVelocity;
use crate::motor::{Motor, MotorFeedback};
use cobs::{CobsDecoder, CobsEncoder};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio::Pio;
use embassy_rp::pwm::Pwm;
use embassy_rp::usb::{Driver, Instance};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Ticker, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use log::{info, warn};

use {defmt_rtt as _, panic_probe as _};

use encoder::{Direction, PioEncoder, PioEncoderProgram};

mod encoder;
mod kinematics;
mod motor;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

/// CHASSIS_VEL_SIGNAL is in robot local coordinate frame
static CHASSIS_VEL_SIGNAL: Signal<CriticalSectionRawMutex, ChassisVelocity> = Signal::new();

static DRIBBLER_CONTROL_SIGNAL: Signal<CriticalSectionRawMutex, i32> = Signal::new();

static ODOM_WATCH_FL: Watch<CriticalSectionRawMutex, i32, 8> = Watch::new();
static ODOM_WATCH_FR: Watch<CriticalSectionRawMutex, i32, 8> = Watch::new();
static ODOM_WATCH_RL: Watch<CriticalSectionRawMutex, i32, 8> = Watch::new();
static ODOM_WATCH_RR: Watch<CriticalSectionRawMutex, i32, 8> = Watch::new();

const PULSES_PER_MM: f32 = 1050. / (48.0 * PI);

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

macro_rules! odom_task {
    ($name:ident, $pin:ty) => {
        #[embassy_executor::task]
        async fn $name(
            mut encoder: $pin,
            watch: &'static Watch<CriticalSectionRawMutex, i32, 8>,
            reverse: bool,
        ) {
            let mut count: i32 = 0;
            let sender = watch.sender();
            loop {
                count += match encoder.read().await {
                    Direction::Clockwise => 1,
                    Direction::CounterClockwise => -1,
                } * (if reverse { -1 } else { 1 });
                sender.send(count);
            }
        }
    };
}

#[embassy_executor::task]
async fn dribbler_task(mut dribbler: Motor) {
    loop {
        dribbler.set_speed(DRIBBLER_CONTROL_SIGNAL.wait().await);
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
    let mut recv = [
        ODOM_WATCH_FL.receiver().unwrap(),
        ODOM_WATCH_RL.receiver().unwrap(),
        ODOM_WATCH_RR.receiver().unwrap(),
        ODOM_WATCH_FR.receiver().unwrap(),
    ];
    loop {
        if let Some(chassis_vel) = CHASSIS_VEL_SIGNAL.try_take() {
            // Compute wheel velocities from chassis velocities
            let wheel_vels = chassis_vel.inverse_kinematics();
            for (i, motor) in motors.iter_mut().enumerate() {
                motor.target = wheel_vels.as_array()[i] as i32;
                info!("motor {} speed {}", i, wheel_vels.as_array()[i] as i32);
            }
        }

        for (motor, signal) in zip(motors.iter_mut(), recv.iter_mut()) {
            motor.update(signal.try_get().unwrap_or(0));
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

    let dribbler = Motor::new(
        Output::new(p.PIN_0, Level::Low),
        Pwm::new_output_b(p.PWM_SLICE0, p.PIN_1, Default::default())
            .split()
            .1
            .unwrap(),
        true,
    );
    spawner.spawn(dribbler_task(dribbler)).unwrap();

    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        sm3,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let prg = PioEncoderProgram::new(&mut common);
    let encoder0 = PioEncoder::new(&mut common, sm0, p.PIN_11, p.PIN_12, &prg, 500_000);
    let encoder1 = PioEncoder::new(&mut common, sm1, p.PIN_19, p.PIN_20, &prg, 500_000);
    let encoder2 = PioEncoder::new(&mut common, sm2, p.PIN_13, p.PIN_14, &prg, 500_000);
    let encoder3 = PioEncoder::new(&mut common, sm3, p.PIN_21, p.PIN_22, &prg, 500_000);

    odom_task!(odom_task_0, PioEncoder<'static, PIO0, 0>);
    odom_task!(odom_task_1, PioEncoder<'static, PIO0, 1>);
    odom_task!(odom_task_2, PioEncoder<'static, PIO0, 2>);
    odom_task!(odom_task_3, PioEncoder<'static, PIO0, 3>);

    spawner
        .spawn(odom_task_0(encoder0, &ODOM_WATCH_FL, true))
        .unwrap();
    spawner
        .spawn(odom_task_1(encoder1, &ODOM_WATCH_RL, true))
        .unwrap();
    spawner
        .spawn(odom_task_2(encoder2, &ODOM_WATCH_RR, false))
        .unwrap();
    spawner
        .spawn(odom_task_3(encoder3, &ODOM_WATCH_FR, false))
        .unwrap();

    spawner
        .spawn(kinematics_task(
            MotorFeedback::new(
                Output::new(p.PIN_2, Level::Low),
                Pwm::new_output_b(p.PWM_SLICE1, p.PIN_3, Default::default())
                    .split()
                    .1
                    .unwrap(),
                0,
                true,
            ),
            MotorFeedback::new(
                Output::new(p.PIN_5, Level::Low),
                Pwm::new_output_a(p.PWM_SLICE2, p.PIN_4, Default::default())
                    .split()
                    .0
                    .unwrap(),
                1,
                false,
            ),
            MotorFeedback::new(
                Output::new(p.PIN_6, Level::Low),
                Pwm::new_output_b(p.PWM_SLICE3, p.PIN_7, Default::default())
                    .split()
                    .1
                    .unwrap(),
                2,
                false,
            ),
            MotorFeedback::new(
                Output::new(p.PIN_9, Level::Low),
                Pwm::new_output_a(p.PWM_SLICE4, p.PIN_8, Default::default())
                    .split()
                    .0
                    .unwrap(),
                3,
                true,
            ),
        ))
        .unwrap();

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("High Scorers");
    config.product = Some("USB Motor Controller");
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
    let mut recv = [
        ODOM_WATCH_FL.receiver().unwrap(),
        ODOM_WATCH_RL.receiver().unwrap(),
        ODOM_WATCH_RR.receiver().unwrap(),
        ODOM_WATCH_FR.receiver().unwrap(),
    ];

    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];

        'outer: for byte in data {
            match decoder.feed(*byte) {
                Err(e) => {
                    warn!("Error parsing packet: {:?}", e);
                }
                Ok(None) => {}
                Ok(Some(n)) => {
                    if n != 16 {
                        warn!("Invalid packet size");
                    } else {
                        let mut control_dst = [0u8; 16];
                        control_dst.copy_from_slice(&decoder.dest()[..16]);
                        let x = i32::from_be_bytes(control_dst[..4].try_into().unwrap());
                        let y = i32::from_be_bytes(control_dst[4..8].try_into().unwrap());
                        let w = i32::from_be_bytes(control_dst[8..12].try_into().unwrap()); // deg/s
                        let dribbler_control =
                            i32::from_be_bytes(control_dst[12..16].try_into().unwrap());

                        info!("Received controls: {} {} {} {}", x, y, w, dribbler_control);

                        CHASSIS_VEL_SIGNAL.signal(ChassisVelocity {
                            x: (x as f32 * PULSES_PER_MM),
                            y: (y as f32 * PULSES_PER_MM),
                            w: (w as f32 * PI / 180.), // into rad/s
                        });

                        DRIBBLER_CONTROL_SIGNAL.signal(dribbler_control);

                        // Return odom of all wheels
                        let mut out_buf = [0u8; 62];
                        let mut encoder = CobsEncoder::new(&mut out_buf);

                        for odom in &mut recv {
                            let Ok(_) = encoder.push(
                                &((odom.try_get().unwrap_or(0) as f32 / PULSES_PER_MM) as i32)
                                    .to_be_bytes(),
                            ) else {
                                warn!("Error encoding data!");
                                continue 'outer;
                            };
                        }

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
