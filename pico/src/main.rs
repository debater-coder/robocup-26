//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates the possibility to send log::info/warn/error/debug! to USB serial port.

#![no_std]
#![no_main]

use cobs::{CobsDecoder, CobsEncoder};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler};
use embassy_rp::watchdog::Watchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use log::{info, warn};
use {defmt_rtt as _, panic_probe as _};

mod motor;

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

static PERIOD_SIGNAL: Signal<CriticalSectionRawMutex, Command> = Signal::new();

#[derive(Debug, PartialEq, Default)]
struct Command {
    /// Velocity forwards in mm/s
    vx: i32,
    /// Velocity left in mm/s
    vy: i32,
    /// Rotational speed anticlockwise in deg/s
    vw: i32,
}

#[embassy_executor::task]
async fn kinematics_task(mut led: Output<'static>) {
    let mut command = Command::default();

    loop {
        if let Some(new_command) = PERIOD_SIGNAL.try_take() {
            if command != new_command {
                command = new_command;
                log::info!("Command changed to {:?}", command);
            }
        }
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
    spawner.spawn(kinematics_task(led)).unwrap();

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
                        let mut freq_dst = [0u8; 4];
                        freq_dst.copy_from_slice(&decoder.dest()[..4]);
                        let freq = u32::from_be_bytes(freq_dst);

                        info!("Received freq: {}", freq);

                        // Send period to led_task
                        let period = Duration::from_hz(freq as u64);
                        PERIOD_SIGNAL.signal(period);

                        // Return period in milliseconds
                        let mut out_buf = [0u8; 62];
                        let mut encoder = CobsEncoder::new(&mut out_buf);
                        let Ok(_) = encoder.push(&period.as_millis().to_be_bytes()) else {
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
