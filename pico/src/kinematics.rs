//! # Citations:
//! - https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf
//! - https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html

use core::{f32::consts::PI, ops::Mul};
use micromath::F32Ext;

// TODO: fix half lengths;
const LX: f32 = ; // Half-length X between wheels
const LY: f32 = ; // Half-length Y between wheels

/// All velocities in mm/s, displacement in mm
#[derive(Debug, Clone, Copy)]
pub struct WheelVector {
    pub fl: f32,
    pub rl: f32,
    pub rr: f32,
    pub fr: f32,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ChassisVector {
    /// in mm/s
    pub x: f32,
    /// in mm/s
    pub y: f32,
    /// in rad/s
    pub w: f32,
}

/// Composes chassis vectors
impl Mul for ChassisVector {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        // (x + iy) * e^(iw) = (x + iy) * (cos w + isin w) = (xcos w - ysin w + i(ycos w + xsin w)
        ChassisVector {
            x: self.x + rhs.x * self.w.cos() - rhs.y * self.w.sin(),
            y: self.y + rhs.y * self.w.cos() + rhs.x * self.w.sin(),
            w: self.w + rhs.w,
        }
    }
}

impl ChassisVector {
    pub fn inverse_kinematics(&self) -> WheelVector {
        let rotation = (LX + LY) * self.w;

        return WheelVector {
            fl: (self.x - self.y - rotation),
            fr: (self.x + self.y + rotation),
            rl: (self.x + self.y - rotation),
            rr: (self.x - self.y + rotation),
        };
    }

    /// Returns as integer array, converts rad -> deg
    pub fn as_int_array(&self) -> [i32; 3] {
        [self.x as i32, self.y as i32, (self.w * 180. / PI) as i32]
    }
}

impl WheelVector {
    pub fn as_array(&self) -> [f32; 4] {
        [self.fl, self.rl, self.rr, self.fr]
    }

    pub fn forwards_kinematics(&self) -> ChassisVector {
        return ChassisVector {
            x: (self.fl + self.rl + self.rr + self.fr) / 4.,
            y: (-self.fl + self.fr + self.rl - self.rr) / 4.,
            w: (-self.fl + self.fr - self.rl + self.rr) / (4. * (LX + LY)),
        };
    }
}
