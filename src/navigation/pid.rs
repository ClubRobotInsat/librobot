//! Contiens les types permettant de manipuler un PID pour le déplacement du robot.

use core::f32;

use crate::navigation::motor::Command;

#[allow(unused_imports)]
use libm::F32Ext;

/// Le PID du robot
pub(crate) struct Pid {
    old_left_count: i64,
    old_right_count: i64,
    pos_kp: f32,
    pos_kd: f32,
    orient_kp: f32,
    orient_kd: f32,
    // La valeur maximale de la commande en sortie
    max_output: u16,
    // La consigne de position du robot exprimée en nombre de tick de roue codeuse
    position_order: i64,
    // La consigne d'angle exprimée en différence de tick de chaque roue codeuse
    orientation_order: i64,
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
            old_left_count: 0,
            old_right_count: 0,
            pos_kp,
            pos_kd,
            orient_kp,
            orient_kd,
            max_output,
            position_order: 0,
            orientation_order: 0,
        }
    }

    /// Mets à jour la consigne en position en terme de tick de roues codeuses
    pub(crate) fn set_position_goal(&mut self, pos: i64) {
        self.position_order = pos;
    }

    pub(crate) fn increment_position_goal(&mut self, pos: i64) {
        self.position_order += pos;
    }

    pub(crate) fn decrement_position_goal(&mut self, pos: i64) {
        self.position_order -= pos;
    }

    /// Mets à jour la consigne en orientation en terme de tick de roues codeuses.
    pub(crate) fn set_orientation_goal(&mut self, orientation: i64) {
        self.orientation_order = orientation;
    }

    /// Incrémente la consigne en orientation de la valeur donnée, en tick de roues codeuses
    pub(crate) fn increment_orientation_goal(&mut self, orientation: i64) {
        self.orientation_order += orientation;
    }

    /// Renvoie la nouvelle consigne à appliquer aux deux roues pour atteindre la commande en position
    fn update_position_command(
        &self,
        left_count: i64,
        right_count: i64,
        left_speed: i64,
        right_speed: i64,
    ) -> f32 {
        let dist = (left_count + right_count) / 2;
        let speed = (left_speed + right_speed) / 2;
        let diff = self.position_order as i64 - dist;
        (diff as f32 * self.pos_kp) - self.pos_kd as f32 * speed as f32
    }

    /// Renvoie les nouvelles consignes à appliquer aux deux roues pour atteindre la commande en orientation
    fn update_orientation_command(
        &self,
        left_count: i64,
        right_count: i64,
        left_speed: i64,
        right_speed: i64,
    ) -> (f32, f32) {
        let orientation = right_count - left_count;
        let speed = right_speed - left_speed;
        let diff = self.orientation_order - orientation;
        let cmd = (diff as f32 * self.orient_kp) - self.orient_kd * speed as f32;
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
    pub(crate) fn update(&mut self, left_ticks: i64, right_ticks: i64) -> (Command, Command) {
        // Mise à jour de la mémoire du PID
        let (new_left_count, new_right_count) = (left_ticks, right_ticks);
        let (left_speed, right_speed) = (
            new_left_count - self.old_left_count,
            new_right_count - self.old_right_count,
        );
        self.old_left_count = new_left_count;
        self.old_right_count = new_right_count;
        // Calcul du PID
        let position_cmd =
            self.update_position_command(new_left_count, new_right_count, left_speed, right_speed);
        let (orientation_cmd_left, orientation_cmd_right) = self.update_orientation_command(
            new_left_count,
            new_right_count,
            left_speed,
            right_speed,
        );

        let left_cmd = position_cmd + orientation_cmd_left;
        let right_cmd = position_cmd + orientation_cmd_right;
        // Truncate resul
        (self.truncate(left_cmd), self.truncate(right_cmd))
    }

    /// Renvoie le but du PID en ticks de roue codeuse sous la forme (gauche,droite).
    pub fn get_qei_goal(&self) -> (i64, i64) {
        (
            self.position_order - self.orientation_order / 2,
            self.position_order + self.orientation_order / 2,
        )
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

        pid.set_position_goal(9000);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(get_qei(&mut qei_left), get_qei(&mut qei_right));
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

        pid.set_position_goal(-9137);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(get_qei(&mut qei_left), get_qei(&mut qei_right));
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

        pid.set_orientation_goal(733);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(get_qei(&mut qei_left), get_qei(&mut qei_right));
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!(
            (motor_left.get_real_position() + 733 / 2).abs() <= 1,
            "{} should be {}",
            motor_left.get_real_position(),
            -733 / 2
        );
        assert!(
            (motor_right.get_real_position() - 733 / 2).abs() <= 1,
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

        pid.set_orientation_goal(-733);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update(get_qei(&mut qei_left), get_qei(&mut qei_right));
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        // Erreur inférieure à 0.1%
        assert!(
            (motor_left.get_real_position() - 733 / 2).abs() <= 1,
            "{} should be {}",
            motor_left.get_real_position(),
            733 / 2
        );
        assert!(
            (motor_right.get_real_position() + 733 / 2).abs() <= 1,
            "{} should be {}",
            motor_right.get_real_position(),
            -733 / 2
        );
    }

}
