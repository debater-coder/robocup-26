//! This example shows how to use the PIO module in the RP2040 to read a quadrature rotary encoder.

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::{PIO0, USB};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_rp::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

macro_rules! handle_encoder {
    ($name:ident, $pin:ty) => {
        #[embassy_executor::task]
        async fn $name(mut encoder: $pin) {
            let mut count = 0;
            loop {
                log::info!("[{}] Count: {}", stringify!($name), count);
                count += match encoder.read().await {
                    Direction::Clockwise => 1,
                    Direction::CounterClockwise => -1,
                };
            }
        }
    };
}
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let Pio {
        mut common,
        sm0,
        sm1,
        sm2,
        sm3,
        ..
    } = Pio::new(p.PIO0, Irqs);

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

    let mut logger_state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create a class for the logger
    let logger_class = CdcAcmClass::new(&mut builder, &mut logger_state, 64);

    // Creates the logger and returns the logger future
    // Note: You'll need to use log::info! afterwards instead of info! for this to work (this also applies to all the other log::* macros)
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    let prg = PioEncoderProgram::new(&mut common);
    let encoder0 = PioEncoder::new(&mut common, sm0, p.PIN_11, p.PIN_12, &prg);
    let encoder1 = PioEncoder::new(&mut common, sm1, p.PIN_19, p.PIN_20, &prg);
    let encoder2 = PioEncoder::new(&mut common, sm2, p.PIN_13, p.PIN_14, &prg);
    let encoder3 = PioEncoder::new(&mut common, sm3, p.PIN_21, p.PIN_22, &prg);

    handle_encoder!(encoder_0, PioEncoder<'static, PIO0, 0>);
    handle_encoder!(encoder_1, PioEncoder<'static, PIO0, 1>);
    handle_encoder!(encoder_2, PioEncoder<'static, PIO0, 2>);
    handle_encoder!(encoder_3, PioEncoder<'static, PIO0, 3>);

    spawner.spawn(encoder_0(encoder0)).unwrap();
    spawner.spawn(encoder_1(encoder1)).unwrap();
    spawner.spawn(encoder_2(encoder2)).unwrap();
    spawner.spawn(encoder_3(encoder3)).unwrap();

    join(usb_fut, log_fut).await;
}
