//! Contiens les types permettant de manipuler un PID pour le déplacement du robot.

use core::f32;
use core::fmt::{Debug, Display, Formatter, Result};

use embedded_hal::digital::OutputPin;
use embedded_hal::PwmPin;

use crate::units::MilliMeter;

#[allow(unused_imports)]
use libm::F32Ext;

/// Le PID du robot basé sur des unités du monde physique, il contient :
/// * un PID basé sur les ticks de roue codeuse
/// * les informations nécessaires pour passer du monde des ticks de roue codeuses au monde physique
pub(crate) struct RealWorldPid {
    internal_pid: Pid,
    coder_radius: MilliMeter,
    inter_axial_length: MilliMeter,
}

/// Les paramètres d'un PID
#[derive(Debug, Default)]
pub struct PIDParameters {
    /// Le rayon d'une roue codeuse
    pub coder_radius: MilliMeter,
    /// La distance entre les roues codeuses
    pub inter_axial_length: MilliMeter,
    /// Le coefficient proportionnel sur la position
    pub pos_kp: f32,
    /// Le coefficient dérivé sur la position
    pub pos_kd: f32,
    /// Le coefficient proportionnel sur l'orientation
    pub orient_kp: f32,
    /// Le coefficient dérivée sur l'orientation
    pub orient_kd: f32,
    /// La valeur maximale en sortie
    pub max_output: u16,
}

impl RealWorldPid {
    /// Crée un nouveau PID à partir de :
    /// *  2 struct de `embedded_hal` wrappés dans des `QeiManager`représentant les encodeurs quadratiques gauche et droite
    /// * les coefficients de l'asservissement,
    /// * la valeur maximale de la consigne en sortie,
    /// * les valeurs physiques du robot :
    ///     * distance interaxe en mm
    ///     * rayon d'une roue codeuse en mm
    pub(crate) fn new(params: &PIDParameters) -> Self {
        RealWorldPid {
            internal_pid: Pid::new(
                params.pos_kp,
                params.pos_kd,
                params.orient_kp,
                params.orient_kd,
                params.max_output,
            ),
            coder_radius: params.coder_radius,
            inter_axial_length: params.inter_axial_length,
        }
    }

    /// Renvoie un tuple contenant la commande à appliqué au moteur gauche et au moteur droit
    pub(crate) fn update(&mut self, left_ticks: i64, right_ticks: i64) -> (Command, Command) {
        self.internal_pid.update(left_ticks, right_ticks)
    }

    /// Ordonne au robot d'avancer de `distance`
    pub(crate) fn forward(&mut self, distance: MilliMeter) {
        let distance_per_wheel_turn =
            self.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;
        let nb_wheel_turn = distance.as_millimeters() as f32 / distance_per_wheel_turn;
        let ticks = 1024.0 * nb_wheel_turn;
        self.internal_pid
            .increment_position_goal(ticks.round() as i64);
    }

    /// Ordonne au robot de reculer de `distance`
    pub(crate) fn backward(&mut self, distance: MilliMeter) {
        let distance_per_wheel_turn =
            self.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;
        let nb_wheel_turn = distance.as_millimeters() as f32 / distance_per_wheel_turn;
        let ticks = 1024.0 * nb_wheel_turn;
        self.internal_pid.decrement_position_goal(ticks as i64);
    }

    /// Ordonne au robot de tourner de `angle` (en milliradians)
    pub(crate) fn rotate(&mut self, angle: i64) {
        let turn_distance = angle as f32 * self.inter_axial_length.as_millimeters() as f32 * 0.001 * 0.5;
        let distance_per_wheel_turn =
            self.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;
        let nb_wheel_turn = turn_distance / distance_per_wheel_turn;
        let ticks = 1024.0 * nb_wheel_turn;
        self.internal_pid.increment_orientation_goal(ticks as i64);
    }
}

/// Un moteur avec ses deux broches : vitesse et direction.
pub struct Motor<MOT, DIR>
where
    MOT: PwmPin<Duty = u16>,
    DIR: OutputPin,
{
    pwm: MOT,
    dir: DIR,
}

impl<MOT, DIR> Debug for Motor<MOT, DIR>
where
    MOT: PwmPin<Duty = u16>,
    DIR: OutputPin,
{
    fn fmt(&self, f: &mut Formatter) -> Result {
        write!(
            f,
            "Motor {{ Dir : Unknown, Pwm : {} }}",
            self.pwm.get_duty()
        )
    }
}

impl<MOT, DIR> Motor<MOT, DIR>
where
    MOT: PwmPin<Duty = u16>,
    DIR: OutputPin,
{
    /// Crée une nouvelle structure de gestion moteur à partir d'une broche de direction et d'une
    /// broche de PWM :
    /// * la PWM commande la vitesse : son duty cycle est proportionnel à la vitesse voulue du
    /// moteur
    /// * la broche d'entrée sortie controle la direction du moteur
    pub fn new(pwm: MOT, dir: DIR) -> Self {
        Motor { pwm, dir }
    }

    /// Applique la commande de direction et de vitesse aux moteurs :
    /// * avancer correspond à un état bas sur la broche de direction
    /// * reculer correspond à un état haut sur la broche de direction
    pub fn apply_command(&mut self, cmd: Command) {
        match cmd {
            Command::Front(pwm) => {
                self.pwm.set_duty(pwm);
                self.dir.set_high();
            }
            Command::Back(pwm) => {
                self.pwm.set_duty(pwm);
                self.dir.set_low();
            }
        }
    }
}

/// Une commande pour un moteur : une direction et une vitesse sur 16 bits (0 : vitesse nulle).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Command {
    /// Le moteur doit avancer à la vitesse fournie
    Front(u16),
    /// Le moteur doit reculer à la vitesse fournie
    Back(u16),
}

impl Display for Command {
    fn fmt(&self, f: &mut Formatter) -> Result {
        match self {
            Command::Front(val) => write!(f, "Forward : {}", val),
            Command::Back(val) => write!(f, "Backward {}", val),
        }
    }
}

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
            self.position_order + self.orientation_order / 2,
            self.position_order - self.orientation_order / 2,
        )
    }
}

#[cfg(test)]
mod test {

    use std::cell::Cell;
    use std::rc::Rc;

    use crate::navigation::pid::{Command, Pid, RealWorldPid, PIDParameters};
    use crate::units::MilliMeter;
    use embedded_hal::Qei;
    use qei::QeiManager;

    #[derive(Debug, Clone, Copy)]
    enum Direction {
        Front,
        Back,
    }

    #[derive(Debug, Clone)]
    struct DummyMotor {
        speed: Rc<Cell<u16>>,
        real_position: Rc<Cell<i64>>,
        wrapped_position: Rc<Cell<u16>>,
        dir: Rc<Cell<Direction>>,
    }

    impl DummyMotor {
        fn new() -> Self {
            DummyMotor {
                speed: Rc::new(Cell::new(0)),
                real_position: Rc::new(Cell::new(0)),
                wrapped_position: Rc::new(Cell::new(0)),
                dir: Rc::new(Cell::new(Direction::Front)),
            }
        }

        fn update(&mut self) {
            match self.dir.get() {
                Direction::Front => {
                    self.real_position
                        .replace(self.real_position.get() + self.speed.get() as i64);
                    self.wrapped_position
                        .replace(self.wrapped_position.get().wrapping_add(self.speed.get()));
                }
                Direction::Back => {
                    self.real_position
                        .replace(self.real_position.get() - self.speed.get() as i64);
                    self.wrapped_position
                        .replace(self.wrapped_position.get().wrapping_sub(self.speed.get()));
                }
            }
        }

        fn apply_command(&mut self, command: Command) {
            match command {
                Command::Front(speed) => {
                    self.speed.replace(speed / 5);
                    self.dir.replace(Direction::Front);
                }
                Command::Back(speed) => {
                    self.speed.replace(speed / 5);
                    self.dir.replace(Direction::Back);
                }
            }
        }

        pub fn get_real_position(&self) -> i64 {
            self.real_position.get()
        }
    }

    impl Qei for DummyMotor {
        type Count = u16;

        /// Returns the current pulse count of the encoder
        fn count(&self) -> Self::Count {
            self.wrapped_position.get() as u16
        }

        /// Returns the count direction
        fn direction(&self) -> embedded_hal::Direction {
            embedded_hal::Direction::Upcounting
        }
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

    #[test]
    fn real_world_pid_forward() {
        let pid_parameters = PIDParameters {
            coder_radius: MilliMeter(30),
            inter_axial_length: MilliMeter(300),
            pos_kp: 1.0,
            pos_kd: 1.0,
            orient_kp: 1.0,
            orient_kd: 1.0,
            max_output: 100,
        };

        let mut pid = RealWorldPid::new(&pid_parameters);
        pid.forward(MilliMeter(100));

        let (goall, goalr) = pid.internal_pid.get_qei_goal();

        assert!(
            (goall - 543).abs() <= 1,
            "{} should be {}",
            goall,
            543
        );
        assert!(
            (goalr - 543).abs() <= 1,
            "{} should be {}",
            goalr,
            543
        );
    }

    #[test]
    fn real_world_pid_rotation() {
        let pid_parameters = PIDParameters {
            coder_radius: MilliMeter(30),
            inter_axial_length: MilliMeter(300),
            pos_kp: 1.0,
            pos_kd: 1.0,
            orient_kp: 1.0,
            orient_kd: 1.0,
            max_output: 100,
        };

        let mut pid = RealWorldPid::new(&pid_parameters);
        pid.rotate(785); // PI / 4 en milliradians

        let (goall, goalr) = pid.internal_pid.get_qei_goal();

        assert!(
            (goall + 640).abs() <= 1,
            "{} should be {}",
            goall,
            -640
        );
        assert!(
            (goalr - 640).abs() <= 1,
            "{} should be {}",
            goalr,
            640
        );
    }
}
