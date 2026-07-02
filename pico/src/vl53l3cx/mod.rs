use core::{cell::OnceCell, slice};

use embassy_rp::{
    i2c::{Blocking, I2c},
    peripherals::I2C0,
};
use embassy_time::{Delay, Instant};
use embedded_hal::delay::DelayNs;
use log::{info, warn};

use crate::vl53l3cx::bindings::{
    Robocup_PlatformInit, Robocup_Platform_t, VL53LX_DataInit, VL53LX_DevData_t, VL53LX_Dev_t,
    VL53LX_Error, VL53LX_GetMeasurementDataReady, VL53LX_GetMultiRangingData,
    VL53LX_MultiRangingData_t, VL53LX_StartMeasurement, VL53LX_WaitDeviceBooted,
};

mod bindings;

static I2C_INSTANCE: OnceCell<I2c<'static, I2C0, Blocking>> = OnceCell::new();

unsafe extern "C" fn write_multi(
    i2c_address: u8,
    index: u16,
    pdata: *mut u8,
    count: u32,
) -> VL53LX_Error {
    let mut tx_buffer = [0u8; 256];

    let total_len = 2 + count as usize;
    if total_len > tx_buffer.len() {
        warn!("write_multi: length too long");
        return -1;
    }

    let index_bytes = index.to_be_bytes();
    tx_buffer[0] = index_bytes[0];
    tx_buffer[1] = index_bytes[1];

    tx_buffer[2..total_len].copy_from_slice(slice::from_raw_parts(pdata, count as usize));
    info!("write_multi: address: {:x}, index: {:x} tx_buffer: {:?}");

    I2C_INSTANCE
        .get()
        .unwrap()
        .blocking_write(address, &tx_buffer[..total_len]);

    return 0;
}

unsafe extern "C" fn read_multi(
    i2c_address: u8,
    index: u16,
    pdata: *mut u8,
    count: u32,
) -> VL53LX_Error {
    let index_bytes = index.to_be_bytes();

    let dest_slice = slice::from_raw_parts_mut(pdata, count as usize);

    match I2C_INSTANCE
        .get()
        .unwrap()
        .blocking_write_read(address, &index_bytes, dest_slice)
    {
        Ok(_) => {
            info!(
                "read_multi: address: {:x}, index: {:x}, read {} bytes",
                address, index, count
            );
            0
        }
        Err(e) => {
            error!("read_multi failed: {:?}", e);
            -1
        }
    }
}

unsafe extern "C" fn wait_us(wait_us: i32) {
    let mut delay = Delay;
    delay.delay_us(u32::try_from(wait_us).unwrap_or(0));
}

unsafe extern "C" fn wait_ms(wait_ms: i32) {
    let mut delay = Delay;
    delay.delay_ms(u32::try_from(wait_ms).unwrap_or(0));
}

unsafe extern "C" fn get_tick_count() -> u32 {
    Instant::now().as_millis() as u32
}

pub fn vl53lx_error_to_result(error: VL53LX_Error) -> Result<(), i8> {
    if error == 0 {
        Ok(())
    } else {
        Err(error)
    }
}

pub fn init() -> Result<VL53LX_Dev_t, VL53LX_Error> {
    let mut platform = Robocup_Platform_t {
        writeMulti: Some(write_multi),
        readMulti: Some(read_multi),
        waitUs: Some(wait_us),
        waitMs: Some(wait_ms),
        getTickCount: Some(get_tick_count),
    };

    let mut device = VL53LX_Dev_t {
        Data: VL53LX_DevData_t::default(),
        i2c_slave_address: 0x29,
        comms_type: 0,
        comms_speed_khz: 100,
        new_data_ready_poll_duration_ms: 1,
    };

    let dev_ptr = core::ptr::from_mut(&mut device);

    unsafe {
        Robocup_PlatformInit(core::ptr::from_mut(&mut platform));

        vl53lx_error_to_result(VL53LX_WaitDeviceBooted(dev_ptr))?;
        vl53lx_error_to_result(VL53LX_DataInit(dev_ptr))?;
        vl53lx_error_to_result(VL53LX_StartMeasurement(dev_ptr))?;
    }

    Ok(device)
}

pub fn get_measurement_data_blocking(
    device: &mut VL53LX_Dev_t,
) -> Result<VL53LX_MultiRangingData_t, VL53LX_Error> {
    let mut new_data_ready = 0;
    let mut status = 0;
    let mut ranging_data = VL53LX_MultiRangingData_t::default();

    let ranging_data_ptr = core::ptr::from_mut(&mut ranging_data);
    let dev_ptr = core::ptr::from_mut(device);

    let mut delay = Delay;

    loop {
        unsafe {
            status =
                VL53LX_GetMeasurementDataReady(dev_ptr, core::ptr::from_mut(&mut new_data_ready));
            if status == 0 && new_data_ready != 0 {
                status = VL53LX_GetMultiRangingData(dev_ptr, ranging_data_ptr);

                info!("ranging data status: {}, data: {:?}", status, ranging_data);

                status = VL53LX_ClearInterruptAndStartMeasurement(dev_ptr);

                info!("clear interupt status: {}", status);

                return OK(ranging_data);
            } else {
                warn!("in loop: status {}, {}", status, new_data_ready);
                delay.delay_ms(1);
            }
        }
    }
}
