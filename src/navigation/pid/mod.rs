mod double;
mod simple;

use core::f32;
#[allow(unused_imports)]
use libm::F32Ext;

pub use self::double::*;
pub use self::simple::*;

#[allow(non_snake_case)]
pub(crate) struct PID {
    kp: f32,
    kd: f32,
    ki: f32,
    I: f32,
    current: f32,
    current_error: f32,
    goal: f32,
    command: f32,
}

impl PID {
    pub(crate) fn new(kp: f32, kd: f32, ki: f32) -> PID {
        PID {
            kp,
            kd,
            ki,
            I: 0.0,
            /// Current value of the controller
            current: 0.0,
            /// Current error (error is current - goal)
            current_error: 0.0,
            goal: 0.0,
            command: 0.0,
        }
    }

    pub(crate) fn set_goal(&mut self, goal: f32) {
        self.goal = goal;
    }

    pub(crate) fn increment_goal(&mut self, inc: f32) {
        self.goal += inc;
    }

    pub(crate) fn get_goal(&self) -> f32 {
        self.goal
    }

    pub(crate) fn get_command(&self) -> f32 {
        self.command
    }

    pub(crate) fn update(&mut self, val: f32) {
        let error = self.goal - val;
        let d_error = error - self.current_error;
        self.I += error + self.current_error;
        self.command = error * self.kp + self.I * self.ki + d_error * self.kd;
        self.current = val;
        self.current_error = error;
    }
}
