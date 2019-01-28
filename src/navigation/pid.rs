//! Contiens les types permettant de manipuler un PID pour le déplacement du robot.

use core::f32;
use core::fmt::{Debug, Display, Formatter, Result};

use qei::QeiManager;

use embedded_hal::digital::OutputPin;
use embedded_hal::{PwmPin, Qei};

use crate::units::MilliMeter;

use crate::navigation::Coord;

#[allow(unused_imports)]
use libm::F32Ext;

/// Le PID du robot basé sur des unités du monde physique, il contient :
/// * un PID basé sur les ticks de roue codeuse
/// * les informations nécessaires pour passer du monde des ticks de roue codeuses au monde physique
#[derive(Debug)]
pub struct RealWorldPid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
    u16: core::convert::From<<R as embedded_hal::Qei>::Count>,
    u16: core::convert::From<<L as embedded_hal::Qei>::Count>,
{
    internal_pid: Pid<L, R>,
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

impl<L, R> RealWorldPid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
    u16: core::convert::From<<R as embedded_hal::Qei>::Count>,
    u16: core::convert::From<<L as embedded_hal::Qei>::Count>,
{
    /// Crée un nouveau PID à partir de :
    /// *  2 struct de `embedded_hal` wrappés dans des `QeiManager`représentant les encodeurs quadratiques gauche et droite
    /// * les coefficients de l'asservissement,
    /// * la valeur maximale de la consigne en sortie,
    /// * les valeurs physiques du robot :
    ///     * distance interaxe en mm
    ///     * rayon d'une roue codeuse en mm
    pub fn new(qei_left: QeiManager<L>, qei_right: QeiManager<R>, params: &PIDParameters) -> Self {
        RealWorldPid {
            internal_pid: Pid::new(
                params.pos_kp,
                params.pos_kd,
                params.orient_kp,
                params.orient_kd,
                params.max_output,
                qei_left,
                qei_right,
            ),
            coder_radius: params.coder_radius,
            inter_axial_length: params.inter_axial_length,
        }
    }

    /// Renvoie un tuple contenant la commande à appliqué au moteur gauche et au moteur droit
    pub fn update(&mut self) -> (Command, Command) {
        self.internal_pid.update()
    }

    /// Ordonne au robot d'avancer de `distance`
    pub fn forward(&mut self, distance: MilliMeter) {
        let distance_per_wheel_turn =
            self.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;
        let nb_wheel_turn = distance.as_millimeters() as f32 / distance_per_wheel_turn;
        let ticks = 1024.0 * nb_wheel_turn;
        self.internal_pid
            .increment_position_goal(ticks.round() as i64);
    }

    /// Ordonne au robot de reculer de `distance`
    pub fn backward(&mut self, distance: MilliMeter) {
        let distance_per_wheel_turn =
            self.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;
        let nb_wheel_turn = distance.as_millimeters() as f32 / distance_per_wheel_turn;
        let ticks = 1024.0 * nb_wheel_turn;
        self.internal_pid.decrement_position_goal(ticks as i64);
    }

    /// Permet de récuperer l'orientation du robot à partir d'une différence de ticks de roue codeuse.
    /// Renvoi des radians
    pub fn get_orientation(&mut self) -> f32 {
        // On a 2pi = un tour de robot, soit 2 * pi radians = inter_axial * 2 * pi mm
        // Donc, nb_tour = 2*inter_axial*pi / ((tick_left - tick_right) * wheel_diameter)
        // On converti une différence de tick en angle
        let (tick_left, tick_right) = self.internal_pid.get_qei_count();
        if tick_left - tick_right == 0 {
            0.0
        } else {
            2.0 * self.inter_axial_length.as_millimeters() as f32 * core::f32::consts::PI
                / ((tick_left - tick_right) as f32 * self.coder_radius.as_millimeters() as f32)
        }
    }

    /// Renvoies la position du robot
    pub fn get_position(&mut self) -> Coord {
        let (tick_left, tick_right) = self.internal_pid.get_qei_count();

        let orientation = self.get_orientation();
        let distance = ((tick_left + tick_right) as f32 / 1024.0 /* x2.0 manquant car on simplifie en haut et en bas */)
            * self.coder_radius.as_millimeters() as f32 * /* x2.0 manquant car on simplifie en haut et en bas */ f32::consts::PI;

        let (sin, cos) = orientation.sin_cos();
        let x = distance * sin;
        let y = distance * cos;
        Coord {
            x: MilliMeter(x.round() as i64),
            y: MilliMeter(y.round() as i64),
        }
    }

    /// Remets à 0 l'origine du robot
    pub fn reset_origin(&mut self) {
        self.internal_pid.reset_origin()
    }

    /// Renvoie les ticks comptés par les roues codeuses de manière brute, sans traitement.
    /// La valeur retournée est sous la forme (gauche,droite).
    pub fn get_qei_ticks(&mut self) -> (i64, i64) {
        self.internal_pid.get_qei_ticks()
    }

    /// Renvoie sous la forme (gauche,droite) le but du PID en terme de ticks de roue codeuse.
    pub fn get_qei_goal(&mut self) -> (i64, i64) {
        self.internal_pid.get_qei_goal()
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
pub(crate) struct Pid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
    u16: core::convert::From<<R as embedded_hal::Qei>::Count>,
    u16: core::convert::From<<L as embedded_hal::Qei>::Count>,
{
    old_left_count: i64,
    old_right_count: i64,
    left_qei: QeiManager<L>,
    right_qei: QeiManager<R>,
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

impl<L, R> Debug for Pid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
    u16: core::convert::From<<R as embedded_hal::Qei>::Count>,
    u16: core::convert::From<<L as embedded_hal::Qei>::Count>,
{
    fn fmt(&self, f: &mut Formatter) -> Result {
        write!(
            f,
            "QeiLeft : {}, QeiRight : {}",
            self.left_qei.count(),
            self.right_qei.count()
        )
    }
}

// Implémentation du PID
impl<L, R> Pid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
    u16: core::convert::From<<R as embedded_hal::Qei>::Count>,
    u16: core::convert::From<<L as embedded_hal::Qei>::Count>,
{
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
        left_qei: QeiManager<L>,
        right_qei: QeiManager<R>,
    ) -> Self {
        Pid {
            old_left_count: 0,
            old_right_count: 0,
            left_qei,
            right_qei,
            pos_kp,
            pos_kd,
            orient_kp,
            orient_kd,
            max_output,
            position_order: 0,
            orientation_order: 0,
        }
    }

    /// Renvoie la valeur en ticks de la distance parcourue par les roues codeuses
    pub(crate) fn get_qei_count(&mut self) -> (i64, i64) {
        (self.left_qei.count(), self.right_qei.count())
    }

    pub(crate) fn reset_origin(&mut self) {
        self.left_qei.reset();
        self.right_qei.reset();
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
    pub(crate) fn update(&mut self) -> (Command, Command) {
        // Mise à jour des QEI QEI
        self.left_qei.sample_unwrap();
        self.right_qei.sample_unwrap();

        // Mise à jour de la mémoire du PID
        let (new_left_count, new_right_count) = (self.left_qei.count(), self.right_qei.count());
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

    /// Renvoie les ticks comptés par les roues codeuses sous la forme
    /// (gauche, droite)
    pub fn get_qei_ticks(&self) -> (i64, i64) {
        (self.left_qei.count(), self.right_qei.count())
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

    use embedded_hal::Qei;
    use crate::navigation::pid::{Command, Pid, RealWorldPid};
    use crate::navigation::*;
    use qei::QeiManager;
    use crate::units::MilliMeter;

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

    #[test]
    fn pid_forward() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = Pid::new(1.0, 1.0, 1.0, 1.0, 800, qei_left, qei_right);

        pid.set_position_goal(9000);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update();
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
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = Pid::new(1.0, 1.0, 1.0, 1.0, 800, qei_left, qei_right);

        pid.set_position_goal(-9137);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update();
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
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = Pid::new(1.0, 1.0, 1.0, 1.0, 800, qei_left, qei_right);

        pid.set_orientation_goal(733);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update();
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
            733 / 2
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
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = Pid::new(1.0, 1.0, 1.0, 1.0, 800, qei_left, qei_right);

        pid.set_orientation_goal(-733);
        for _ in 0..999 {
            let (cmdl, cmdr) = pid.update();
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
            733 / 2
        );
    }

    //#[test]
    fn real_world_pid_forward() {
        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let params = PIDParameters {
            coder_radius: MilliMeter(31),
            inter_axial_length: MilliMeter(223),
            pos_kp: 1.0,
            pos_kd: 1.0,
            orient_kp: 1.0,
            orient_kd: 1.0,
            max_output: 800,
        };
        let mut pid = RealWorldPid::new(qei_left, qei_right, &params);
        pid.forward(MilliMeter(50));
        for _ in 0..500 {
            let (cmdl, cmdr) = pid.update();
            motor_left.apply_command(cmdl);
            motor_right.apply_command(cmdr);
            motor_left.update();
            motor_right.update();
        }
        assert_eq!(
            Coord {
                x: MilliMeter(0),
                y: MilliMeter(50)
            },
            pid.get_position()
        );
    }

}
