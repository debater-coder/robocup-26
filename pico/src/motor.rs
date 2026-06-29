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
        self.speed = speed.clamp(-100, 100);
        self.dir.set_level(if (speed > 0) != self.reversed {
            Level::High
        } else {
            Level::Low
        });

        let abs_speed = self.speed.abs();
        self.pwm
            .set_duty_cycle_percent(if abs_speed > 10 { abs_speed } else { 0 } as u8)
            .unwrap();
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
    k_forward: f32,
    /// in pulses
    pub last_diff: i32,
}

impl MotorFeedback {
    pub fn new(dir: Output<'static>, pwm: PwmOutput<'static>, id: u32, reversed: bool) -> Self {
        let mut pid = Pid::new(0.0, 100.0);

        pid.p(0.03, 100.0);
        pid.i(0.003, 10.0);
        pid.d(0.003, 10.0);

        MotorFeedback {
            motor: Motor::new(dir, pwm, reversed),
            pid,
            target: 0,
            last_instant: Instant::now(),
            last_odom: 0,
            motor_id: id,
            k_forward: 0.030769231,
            last_diff: 0,
        }
    }

    /// Call at 20Hz
    pub fn update(&mut self, odom: i32) {
        self.last_diff = odom - self.last_odom; // This is a signed integer for direction
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

        // Hard cuttoff at target 0
        if self.target == 0 {
            self.motor.set_speed(0);
            return;
        }

        // Pulses / s
        let speed = (self.last_diff * 1000) / elapsed.as_millis() as i32;
        info!(
            "[SETPOINT_SPEED_{id}]: {setpoint} | [MEASURED_SPEED_{id}]: {measured}",
            id = self.motor_id,
            setpoint = self.target,
            measured = speed,
        );

        let feed_forward = self.target as f32 * self.k_forward;
        let pid_out = self.pid.next_control_output(speed as f32).output;
        let control_out = feed_forward as i32 + pid_out as i32;

        info!("[CONTROL_OUT_{}]: {}", self.motor_id, control_out);
        info!("[PID_OUT_{}]: {}", self.motor_id, pid_out);

        self.motor.set_speed(control_out);
    }
}
