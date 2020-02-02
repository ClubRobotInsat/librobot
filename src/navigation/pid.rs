//! Contiens les types permettant de manipuler un PID pour le déplacement du robot.

use core::f32;

use crate::navigation::motor::Command;

#[allow(unused_imports)]
use libm::F32Ext;

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

/// Controlleur composé d'un asservissement en position et d'un
/// asservissement en angle.
pub(crate) struct PolarController {
    linear_control: PID,
    angular_control: PID,
    linear_speed_control: PID,
    angular_speed_control: PID,

    current_lin_command: f32,
    current_ang_command: f32,

    max_output: u16,
    max_angle_output: u16,

    /// Time between updates, in seconds
    te: f32,
    max_lin_speed: f32,
    max_ang_speed: f32,
    max_lin_acc: f32,
    max_ang_acc: f32,
    /// Si `false` le robot n'est pas asservi en longitudinal
    linear_control_enabled: bool,
    /// Si `false` le robot n'est pas asservi en angulaire
    angular_control_enabled: bool,

    pos_kd: f32,
    orient_kd: f32,
}

impl PolarController {
    pub(crate) fn new(
        pos_kp: f32,
        pos_ki: f32,
        pos_kd: f32,
        orient_kp: f32,
        orient_ki: f32,
        orient_kd: f32,
        pos_speed_kp: f32,
        orient_speed_kp: f32,
        max_output: u16,
        max_angle_output: u16,
        te: f32,
        max_lin_speed: f32,
        max_ang_speed: f32,
        max_lin_acc: f32,
        max_ang_acc: f32,
    ) -> Self {
        PolarController {
            linear_control: PID::new(pos_kp, pos_kd, pos_ki),
            angular_control: PID::new(orient_kp, orient_kd, orient_ki),
            linear_speed_control: PID::new(pos_speed_kp, 0.0, 0.0),
            angular_speed_control: PID::new(orient_speed_kp, 0.0, 0.0),
            current_lin_command: 0.0,
            current_ang_command: 0.0,
            max_output,
            max_angle_output,
            te,
            max_lin_speed,
            max_ang_speed,
            max_lin_acc: max_lin_acc / 1000.0,
            max_ang_acc: max_ang_acc / 1000.0,
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

    pub(crate) fn set_max_speed(&mut self, lin_speed: f32, ang_speed: f32) {
        self.max_lin_speed = lin_speed;
        self.max_ang_speed = ang_speed;
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

    pub(self) fn clamp_speed(
        &self,
        speed: f32,
        last_speed: f32,
        max_speed: f32,
        max_acc: f32,
    ) -> f32 {
        let upbound = last_speed + max_acc * self.te;
        let lowbound = last_speed - max_acc * self.te;
        Self::clamp(Self::clamp(speed, lowbound, upbound), -max_speed, max_speed)
    }

    pub(crate) fn clamp(val: f32, min: f32, max: f32) -> f32 {
        if val > max {
            max
        } else if val < min {
            min
        } else {
            val
        }
    }

    pub(crate) fn update(&mut self, left_dist: f32, right_dist: f32) -> (Command, Command) {
        // Mise à jour de la mémoire du PID
        let lin_val = (left_dist + right_dist) / 2.0;
        let ang_val = right_dist - left_dist;

        let lin_speed = (lin_val - self.linear_control.current) / self.te;
        let ang_speed = (ang_val - self.angular_control.current) / self.te;

        // Suppression du Kd si l'erreur est inferieure à un certain seuil
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
        self.linear_control.update(lin_val);
        self.angular_control.update(ang_val);

        let lin_speed_goal = self.clamp_speed(
            self.linear_control.get_command(),
            lin_speed,
            self.max_lin_speed,
            self.max_lin_acc,
        );
        let ang_speed_goal = self.clamp_speed(
            self.angular_control.get_command(),
            ang_speed,
            self.max_ang_speed,
            self.max_ang_acc,
        );

        self.linear_speed_control.set_goal(lin_speed_goal);
        self.angular_speed_control.set_goal(ang_speed_goal);

        self.linear_speed_control.update(lin_speed);
        self.angular_speed_control.update(ang_speed);

        self.current_lin_command = if self.linear_control_enabled {
            Self::clamp(
                self.current_lin_command + self.linear_speed_control.get_command(),
                -(self.max_output as f32),
                self.max_output as f32,
            )
        } else {
            0.0
        };
        self.current_ang_command = if self.angular_control_enabled {
            Self::clamp(
                self.current_ang_command + self.angular_speed_control.get_command(),
                -(self.max_angle_output as f32),
                self.max_angle_output as f32,
            )
        } else {
            0.0
        };

        // Truncate result
        (
            Command::truncate(
                self.current_lin_command - self.current_ang_command,
                self.max_output,
            ),
            Command::truncate(
                self.current_lin_command + self.current_ang_command,
                self.max_output,
            ),
        )
    }
}

#[cfg(test)]
mod test {
    use embedded_hal::Qei;
    use qei::QeiManager;

    use crate::navigation::motor::{test::DummyMotor, Command};
    use crate::navigation::pid::PolarController;

    fn create_controller() -> PolarController {
        PolarController::new(
            0.01, 0.0, 0.0, 0.01, 0.0, 0.0, 30.0, 30.0, 800, 800, 1.0, 50.0, 50.0, 100.0, 100.0,
        )
    }

    fn get_qei<T>(qei: &mut QeiManager<T>) -> i64
    where
        T: Qei,
        u16: core::convert::From<<T as embedded_hal::Qei>::Count>,
    {
        qei.sample_unwrap();
        qei.count() as i64
    }

    #[test]
    fn clamp_speed() {
        let pid = create_controller();

        assert!((pid.clamp_speed(9000.0, 0.0, 50.0, 100.0) - 50.0).abs() < 0.001);
        assert!((pid.clamp_speed(9000.0, 0.0, 50.0, 10.0) - 10.0).abs() < 0.001);
    }

    #[test]
    fn pid_forward() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let mut qei_left = QeiManager::new(motor_left.clone());
        let mut qei_right = QeiManager::new(motor_right.clone());
        let mut pid = create_controller();

        pid.set_linear_goal(9000.0);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(
                get_qei(&mut qei_left) as f32,
                get_qei(&mut qei_right) as f32,
            );
            // println!("{} {}", motor_left.get_real_position(), pid.linear_speed_control.goal);
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
        let mut pid = create_controller();

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
        let mut pid = create_controller();

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
        let mut pid = create_controller();

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
