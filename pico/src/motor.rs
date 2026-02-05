use embassy_rp::{
    gpio::{Level, Output},
    pwm::{PwmOutput, SetDutyCycle},
};
use embassy_time::{Duration, Instant};
use log::warn;
use pid::Pid;

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

        self.pwm.set_duty_cycle_percent(speed.abs() as u8).unwrap();
    }

    pub fn get_speed(&self) -> i32 {
        self.speed
    }
}

pub struct MotorFeedback {
    motor: Motor,
    pid: Pid<f32>,
    /// target speed in pulses/s
    pub target: i32,
    last_instant: Instant,
    last_odom: u32,
}

impl MotorFeedback {
    pub fn new(dir: Output<'static>, pwm: PwmOutput<'static>) -> Self {
        let mut pid = Pid::new(0.0, 100.0);

        pid.p(10.0, 100.0);

        MotorFeedback {
            motor: Motor::new(dir, pwm),
            pid,
            target: 0,
            last_instant: Instant::now(),
            last_odom: 0,
        }
    }

    /// Call at 20Hz
    pub fn update(&mut self, odom: u32) {
        let odom_diff = odom - self.last_odom;
        let elapsed = self.last_instant.elapsed();
        self.last_instant = Instant::now();
        self.last_odom = odom;

        if elapsed > Duration::from_millis(75) {
            warn!("Too long between motor feedback");
            return;
        }

        if elapsed < Duration::from_millis(25) {
            warn!("Too short between motor feedback");
            return;
        }

        // Pulses / s
        let mut speed = (odom_diff as i32 * 1000) / elapsed.as_millis() as i32;

        if self.target < 0 {
            speed *= -1;
        }

        let control = self.pid.next_control_output(speed as f32);

        self.motor.set_speed(control.output as i32);
    }
}
