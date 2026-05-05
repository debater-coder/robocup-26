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
    motor_id: u32,
    pid: Pid<f32>,
    /// target speed in pulses/s
    pub target: i32,
    last_instant: Instant,
    last_odom: i32,
}

impl MotorFeedback {
    pub fn new(dir: Output<'static>, pwm: PwmOutput<'static>, id: u32, reversed: bool) -> Self {
        let mut pid = Pid::new(0.0, 100.0);

        pid.p(10.0, 100.0);

        MotorFeedback {
            motor: Motor::new(dir, pwm, reversed),
            pid,
            target: 0,
            last_instant: Instant::now(),
            last_odom: 0,
            motor_id: id,
        }
    }

    /// Call at 20Hz
    pub fn update(&mut self, odom: i32) {
        let odom_diff = odom - self.last_odom; // This is a signed integer for direction
        let elapsed = self.last_instant.elapsed();
        self.last_instant = Instant::now();
        self.last_odom = odom;
        self.pid.setpoint(self.target as f32);

        if elapsed > Duration::from_millis(75) {
            warn!(
                "Too long between motor feedback, motor id: {}",
                self.motor_id
            );
            return;
        }

        if elapsed < Duration::from_millis(25) {
            warn!(
                "Too short between motor feedback, motor id: {}",
                self.motor_id
            );
            return;
        }

        // Pulses / s
        let speed = (odom_diff * 1000) / elapsed.as_millis() as i32;
        info!(
            "setpoint: {}, measured: {}, odom_diff: {}, elapsed: {}, input odom: {}",
            self.target,
            speed,
            odom_diff,
            elapsed.as_millis(),
            odom
        );

        let control = self.pid.next_control_output(speed as f32);
        info!("Motor id: {}, control {:?}", self.motor_id, control);

        self.motor.set_speed(control.output as i32);
        self.motor.set_speed(self.target);
    }
}
