use embassy_rp::{
    gpio::{Level, Output},
    pwm::{PwmOutput, SetDutyCycle},
};

pub struct Motor {
    dir: Output<'static>,
    pwm: PwmOutput<'static>,
    /// Speed value from -100 to 100
    speed: i32,
}

impl Motor {
    pub fn new(dir: Output<'static>, pwm: PwmOutput<'static>) -> Self {
        Motor { dir, pwm, speed: 0 }
    }

    pub fn set_speed(&mut self, speed: i32) {
        self.speed = speed;
        self.dir
            .set_level(if speed > 0 { Level::High } else { Level::Low });

        self.pwm.set_duty_cycle_percent(speed.abs() as u8);
    }

    pub fn get_speed(&self) -> i32 {
        self.speed
    }
}
