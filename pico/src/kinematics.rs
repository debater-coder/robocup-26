//! # Citations:
//! - https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
//! - https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html

const LX: f32 = 0.05; // Half-length X between wheels
const LY: f32 = 0.05; // Half-length Y between wheels

/// All velocities in mm/s
#[derive(Debug, Clone, Copy)]
pub struct WheelVelocities {
    pub fl: f32,
    pub rl: f32,
    pub rr: f32,
    pub fr: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct ChassisVelocity {
    /// in mm/s
    pub x: f32,
    /// in mm/s
    pub y: f32,
    /// in rad/s
    pub w: f32,
}

impl ChassisVelocity {
    pub fn inverse_kinematics(&self) -> WheelVelocities {
        let rotation = (LX + LY) * self.w;

        return WheelVelocities {
            fl: (self.x - self.y - rotation),
            rl: (self.x + self.y + rotation),
            rr: (self.x + self.y - rotation),
            fr: (self.x - self.y + rotation),
        };
    }
}

impl WheelVelocities {
    pub fn as_array(&self) -> [f32; 4] {
        [self.fl, self.rl, self.rr, self.fr]
    }

    pub fn forwards_kinematics(&self) -> ChassisVelocity {
        return ChassisVelocity {
            x: (self.fl + self.rl + self.rr + self.fr) / 4.,
            y: (-self.fl + self.rl + self.rr - self.fr) / 4.,
            w: (-self.fl + self.rl - self.rr + self.fr) / (4. * (LX + LY)),
        };
    }
}
