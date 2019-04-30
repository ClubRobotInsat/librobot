//! Contiens les types permettant de manipuler un PID pour le déplacement du robot.

use core::f32;

use crate::navigation::motor::Command;

#[allow(unused_imports)]
use libm::F32Ext;

/// Le PID du robot
pub(crate) struct Pid {
    old_left_dist: f32,
    old_right_dist: f32,
    pos_kp: f32,
    pos_kd: f32,
    orient_kp: f32,
    orient_kd: f32,
    // La valeur maximale de la commande en sortie
    max_output: u16,
    // La consigne de la roue gauche exprimée en millimètres
    left_goal: f32,
    // La consigne de la roue droite exprimée en millimètres
    right_goal: f32,
}

// Implémentation du PID
impl Pid {
    /// Crée un nouveau PID à partir :
    /// * des coefficients de l'asservissement
    /// * d'une valeur maximale de sortie (la valeur du registre ARR du timer qui gère la PWM par
    /// exemple)
    /// * deux roues codeuses
    pub(crate) fn new(
        pos_kp: f32,
        pos_kd: f32,
        orient_kp: f32,
        orient_kd: f32,
        max_output: u16,
    ) -> Self {
        Pid {
            old_left_dist: 0.0,
            old_right_dist: 0.0,
            pos_kp,
            pos_kd,
            orient_kp,
            orient_kd,
            max_output,
            left_goal: 0.0,
            right_goal: 0.0,
        }
    }

    pub(crate) fn set_goal(&mut self, left: f32, right: f32) {
        self.left_goal = left;
        self.right_goal = right;
    }

    pub(crate) fn increment_goal(&mut self, left: f32, right: f32) {
        self.left_goal += left;
        self.right_goal += right;
    }

    /// Renvoie la nouvelle consigne à appliquer aux deux roues pour atteindre la commande en position
    fn update_position_command(
        &self,
        left_dist: f32,
        right_dist: f32,
        left_speed: f32,
        right_speed: f32,
    ) -> f32 {
        let dist = (left_dist + right_dist) / 2.;
        let speed = (left_speed + right_speed) / 2.;
        let position_order = (self.left_goal + self.right_goal) / 2.;
        let diff = position_order - dist;
        (diff * self.pos_kp) - self.pos_kd * speed
    }

    /// Renvoie les nouvelles consignes à appliquer aux deux roues pour atteindre la commande en orientation
    fn update_orientation_command(
        &self,
        left_dist: f32,
        right_dist: f32,
        left_speed: f32,
        right_speed: f32,
    ) -> (f32, f32) {
        let orientation = right_dist - left_dist;
        let speed = right_speed - left_speed;
        let orientation_order = self.right_goal - self.left_goal;
        let diff = orientation_order - orientation;
        let cmd = (diff * self.orient_kp) - self.orient_kd * speed;
        (-cmd, cmd)
    }

    fn truncate(&self, val: f32) -> Command {
        if val.is_sign_positive() {
            if val > f32::from(self.max_output) {
                Command::Front(self.max_output)
            } else {
                Command::Front(val as u16)
            }
        } else if -val > f32::from(self.max_output) {
            Command::Back(self.max_output)
        } else {
            Command::Back((-val) as u16)
        }
    }

    /// Renvoie la nouvelle consigne à appliquer aux roues pour le pid
    /// L'algorithme est issue de [cette](https://www.rcva.fr/10-ans-dexperience/9/) page internet.
    pub(crate) fn update(&mut self, left_dist: f32, right_dist: f32) -> (Command, Command) {
        // Mise à jour de la mémoire du PID
        let (new_left_dist, new_right_dist) = (left_dist, right_dist);
        let (left_speed, right_speed) = (
            new_left_dist - self.old_left_dist,
            new_right_dist - self.old_right_dist,
        );
        self.old_left_dist = new_left_dist;
        self.old_right_dist = new_right_dist;
        // Calcul du PID
        let position_cmd =
            self.update_position_command(new_left_dist, new_right_dist, left_speed, right_speed);
        let (orientation_cmd_left, orientation_cmd_right) = self.update_orientation_command(
            new_left_dist,
            new_right_dist,
            left_speed,
            right_speed,
        );

        let left_cmd = position_cmd + orientation_cmd_left;
        let right_cmd = position_cmd + orientation_cmd_right;
        // Truncate resul
        (self.truncate(left_cmd), self.truncate(right_cmd))
    }

    /// Renvoie le but du PID en mm sous la forme (gauche,droite).
    pub fn get_qei_goal(&self) -> (f32, f32) {
        (self.left_goal, self.right_goal)
    }
}

#[cfg(test)]
mod test {
    use embedded_hal::Qei;
    use qei::QeiManager;

    use crate::navigation::motor::test::DummyMotor;
    use crate::navigation::pid::Pid;

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
        let mut pid = Pid::new(1.0, 1.0, 1.0, 1.0, 800);

        pid.set_goal(9000.0, 9000.0);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(get_qei(&mut qei_left) as f32, get_qei(&mut qei_right) as f32);
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!((motor_left.get_real_position() - 9000).abs() <= 9);
        assert!((motor_right.get_real_position() - 9000).abs() <= 9);
    }

    #[test]
    fn pid_backward() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let mut qei_left = QeiManager::new(motor_left.clone());
        let mut qei_right = QeiManager::new(motor_right.clone());
        let mut pid = Pid::new(1.0, 1.0, 1.0, 1.0, 800);

        pid.set_goal(-9137.0, -9137.0);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(get_qei(&mut qei_left) as f32, get_qei(&mut qei_right) as f32);
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!((motor_left.get_real_position() + 9137).abs() <= 9);
        assert!((motor_right.get_real_position() + 9137).abs() <= 9);
    }

    #[test]
    fn pid_rotation_left() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let mut qei_left = QeiManager::new(motor_left.clone());
        let mut qei_right = QeiManager::new(motor_right.clone());
        let mut pid = Pid::new(1.0, 1.0, 1.0, 1.0, 800);

        pid.set_goal(-733. / 2., 733. / 2.);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(get_qei(&mut qei_left) as f32, get_qei(&mut qei_right) as f32);
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
        let mut pid = Pid::new(1.0, 1.0, 1.0, 1.0, 800);

        pid.set_goal(733. / 2., -733. / 2.);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(get_qei(&mut qei_left) as f32, get_qei(&mut qei_right) as f32);
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
