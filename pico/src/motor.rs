use embassy_rp::{
    gpio::{Level, Output},
    pwm::{PwmOutput, SetDutyCycle},
};
use embassy_time::{Duration, Instant};
use log::{info, warn};
use pid::Pid;

pub struct Motor {
    dir: Output<'static>,
    pwm: PwmOutput<'static>,
    /// Speed value from -100 to 100
    speed: i32,
    reversed: bool,
}

impl Motor {
    pub fn new(dir: Output<'static>, pwm: PwmOutput<'static>, reversed: bool) -> Self {
        Motor {
            dir,
            pwm,
            speed: 0,
            reversed,
        }
    }

    pub fn set_speed(&mut self, speed: i32) {
        self.speed = speed;
        self.dir.set_level(if (speed > 0) != self.reversed {
            Level::High
        } else {
            Level::Low
        });

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
    last_odom: i32,
}

impl MotorFeedback {
    pub fn new(dir: Output<'static>, pwm: PwmOutput<'static>, reversed: bool) -> Self {
        let mut pid = Pid::new(0.0, 100.0);

        pid.p(10.0, 100.0);

        MotorFeedback {
            motor: Motor::new(dir, pwm, reversed),
            pid,
            target: 0,
            last_instant: Instant::now(),
            last_odom: 0,
        }
    }

    /// Call at 20Hz
    pub fn update(&mut self, odom: i32) {
        self.motor.set_speed(self.target);
    }
}
