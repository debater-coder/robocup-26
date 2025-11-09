//! This example shows how to use USB (Universal Serial Bus) in the RP2040 chip.
//!
//! This creates the possibility to send log::info/warn/error/debug! to USB serial port.

#![no_std]
#![no_main]

use cobs::CobsDecoder;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, Instance, InterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use log::{info, warn};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

static PERIOD_SIGNAL: Signal<CriticalSectionRawMutex, Duration> = Signal::new();

fn update_period(period: &mut Duration) {
    if let Some(duration) = PERIOD_SIGNAL.try_take() {
        *period = duration;
        log::info!("Period changed to {:?} millis", duration.as_millis());
    }
}

#[embassy_executor::task]
async fn logger_task(class: CdcAcmClass<'static, Driver<'static, USB>>) {
    embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, class).await;
}

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>) {
    let mut period: Duration = Duration::from_secs(2);

    loop {
        info!("led on!");
        led.set_high();
        Timer::after(period / 2).await;

        update_period(&mut period);

        info!("led off!");
        led.set_low();
        Timer::after(period / 2).await;

        update_period(&mut period);
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let led = Output::new(p.PIN_10, Level::Low);
    spawner.spawn(led_task(led)).unwrap();

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

    let echo_fut = async {
        loop {
            class.wait_connection().await;
            log::info!("Connected");
            let _ = echo(&mut class).await;
            log::info!("Disconnected");
        }
    };

    join(usb_fut, join(echo_fut, log_fut)).await;
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

async fn echo<'d, T: Instance + 'd>(
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

                        let period = Duration::from_hz(freq as u64);
                        PERIOD_SIGNAL.signal(period);
                    }
                }
            }
        }

        info!("data: {:?}", data);

        class.write_packet(data).await?;
    }
}
