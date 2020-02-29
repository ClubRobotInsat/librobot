//! Contiens les types permettant de manipuler un PID pour le déplacement du robot.

use core::f32;

use crate::navigation::motor::Command;
use super::PID;

#[allow(unused_imports)]
use libm::F32Ext;

/// Controlleur composé d'un asservissement en position et d'un
/// asservissement en angle.
pub(crate) struct SimplePolarController {
    linear_control: PID,
    angular_control: PID,
    max_output: u16,
    max_angle_output: u16,
    /// Si `false` le robot n'est pas asservi en longitudinal
    linear_control_enabled: bool,
    /// Si `false` le robot n'est pas asservi en angulaire
    angular_control_enabled: bool,
    pos_kd: f32,
    orient_kd: f32,
}

impl SimplePolarController {
    pub(crate) fn new(
        pos_kp: f32,
        pos_kd: f32,
        pos_ki: f32,
        orient_kp: f32,
        orient_kd: f32,
        orient_ki: f32,
        max_output: u16,
        max_angle_output: u16,
    ) -> Self {
        SimplePolarController {
            linear_control: PID::new(pos_kp, pos_kd, pos_ki),
            angular_control: PID::new(orient_kp, orient_kd, orient_ki),
            max_output,
            max_angle_output,
            linear_control_enabled: true,
            angular_control_enabled: true,
            pos_kd,
            orient_kd,
        }
    }

    pub(crate) fn enable_control(&mut self, lin_ctrl: bool, ang_ctrl: bool) {
        self.linear_control_enabled = lin_ctrl;
        self.angular_control_enabled = ang_ctrl;
    }

    pub(crate) fn set_max_output(&mut self, max_output: u16) {
        self.max_output = max_output;
    }

    pub(crate) fn set_left_right_goal(&mut self, left: f32, right: f32) {
        self.linear_control.set_goal((left + right) / 2.);
        self.angular_control.set_goal(right - left);
    }

    pub(crate) fn set_linear_goal(&mut self, goal: f32) {
        self.linear_control.set_goal(goal);
    }

    pub(crate) fn increment_linear_goal(&mut self, inc: f32) {
        self.linear_control.increment_goal(inc);
    }

    pub(crate) fn set_angular_goal(&mut self, goal: f32) {
        self.angular_control.set_goal(goal);
    }

    pub(crate) fn increment_angular_goal(&mut self, inc: f32) {
        self.angular_control.increment_goal(inc);
    }

    pub(crate) fn get_left_right_goal(&self) -> (f32, f32) {
        let (lin, ang) = self.get_lin_ang_goal();
        (lin - ang / 2.0, lin + ang / 2.0)
    }

    pub(crate) fn get_lin_ang_goal(&self) -> (f32, f32) {
        (
            self.linear_control.get_goal(),
            self.angular_control.get_goal(),
        )
    }

    pub(crate) fn clamp(val: f32, threshold: f32) -> f32 {
        if val > threshold {
            threshold
        } else if val < -threshold {
            -threshold
        } else {
            val
        }
    }

    pub(crate) fn update(&mut self, left_dist: f32, right_dist: f32) -> (Command, Command) {
        // Mise à jour de la mémoire du PID
        let lin_val = (left_dist + right_dist) / 2.0;
        let ang_val = right_dist - left_dist;

        self.linear_control.update(lin_val);
        self.angular_control.update(ang_val);

        self.linear_control.kd = if self.linear_control.current_error.abs() < 5.0 {
            0.0
        } else {
            self.pos_kd
        };
        self.angular_control.kd = if self.angular_control.current_error.abs() < 8.726646 {
            0.0
        } else {
            self.orient_kd
        };
        // Calcul du PID
        let position_cmd = if self.linear_control_enabled {
            Self::clamp(self.linear_control.get_command(), self.max_output as f32)
        } else {
            0.0
        };
        let orientation_cmd = if self.angular_control_enabled {
            Self::clamp(
                self.angular_control.get_command(),
                self.max_angle_output as f32,
            )
        } else {
            0.0
        };

        // Truncate result
        (
            Command::truncate(position_cmd - orientation_cmd, self.max_output),
            Command::truncate(position_cmd + orientation_cmd, self.max_output),
        )
    }
}

#[cfg(test)]
mod test {
    use embedded_hal::Qei;
    use qei::QeiManager;

    use crate::navigation::motor::test::DummyMotor;
    use crate::navigation::pid::SimplePolarController;

    fn get_qei<T>(qei: &mut QeiManager<T>) -> i64
    where
        T: Qei,
        u16: core::convert::From<<T as embedded_hal::Qei>::Count>,
    {
        qei.sample_unwrap();
        qei.count() as i64
    }

    #[test]
    fn pid_forward() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let mut qei_left = QeiManager::new(motor_left.clone());
        let mut qei_right = QeiManager::new(motor_right.clone());
        let mut pid = SimplePolarController::new(1.0, 1.0, 0.1, 1.0, 1.0, 0.1, 800, 800);

        pid.set_linear_goal(9000.0);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(
                get_qei(&mut qei_left) as f32,
                get_qei(&mut qei_right) as f32,
            );
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!(
            (motor_left.get_real_position() - 9000).abs() <= 9,
            "{} should be {}",
            motor_left.get_real_position(),
            9000
        );
        assert!(
            (motor_right.get_real_position() - 9000).abs() <= 9,
            "{} should be {}",
            motor_left.get_real_position(),
            9000
        );
    }

    #[test]
    fn pid_backward() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let mut qei_left = QeiManager::new(motor_left.clone());
        let mut qei_right = QeiManager::new(motor_right.clone());
        let mut pid = SimplePolarController::new(1.0, 1.0, 0.1, 1.0, 1.0, 0.1, 800, 800);

        pid.set_linear_goal(-9137.0);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(
                get_qei(&mut qei_left) as f32,
                get_qei(&mut qei_right) as f32,
            );
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!(
            (motor_left.get_real_position() + 9137).abs() <= 9,
            "{} should be {}",
            motor_left.get_real_position(),
            -9137
        );
        assert!(
            (motor_right.get_real_position() + 9137).abs() <= 9,
            "{} should be {}",
            motor_left.get_real_position(),
            -9137
        );
    }

    #[test]
    fn pid_rotation_left() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let mut qei_left = QeiManager::new(motor_left.clone());
        let mut qei_right = QeiManager::new(motor_right.clone());
        let mut pid = SimplePolarController::new(1.0, 1.0, 0.1, 1.0, 1.0, 0.1, 800, 800);

        pid.set_angular_goal(733.);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(
                get_qei(&mut qei_left) as f32,
                get_qei(&mut qei_right) as f32,
            );
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!(
            (motor_left.get_real_position() + 733 / 2).abs() <= 2,
            "{} should be {}",
            motor_left.get_real_position(),
            -733 / 2
        );
        assert!(
            (motor_right.get_real_position() - 733 / 2).abs() <= 2,
            "{} should be {}",
            motor_right.get_real_position(),
            733 / 2
        );
    }

    #[test]
    fn pid_rotation_right() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let mut qei_left = QeiManager::new(motor_left.clone());
        let mut qei_right = QeiManager::new(motor_right.clone());
        let mut pid = SimplePolarController::new(1.0, 1.0, 0.1, 1.0, 1.0, 0.1, 800, 800);

        pid.set_angular_goal(-733.);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(
                get_qei(&mut qei_left) as f32,
                get_qei(&mut qei_right) as f32,
            );
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!(
            (motor_left.get_real_position() - 733 / 2).abs() <= 2,
            "{} should be {}",
            motor_left.get_real_position(),
            733 / 2
        );
        assert!(
            (motor_right.get_real_position() + 733 / 2).abs() <= 2,
            "{} should be {}",
            motor_right.get_real_position(),
            -733 / 2
        );
    }

}
