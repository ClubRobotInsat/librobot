//! Ce module contient tout le code qui permets au robot de se déplacer sur la table.
//!
//! Pour rappel, la carte est câblée ainsi :
//!```text
//!                                                     ^----+Sortie du pont H
//!                                                    |
//!                                                    |
//!                                   +--------------+ | +------------+ +--+   ++
//!+-----------------------+          | Pont H       +-v-+ Moteur DC  | |  |   ||   +-----------+
//!|    Carte de commande  |          |              |   |            | |  |   ||   |Codeur     |
//!|                       |          | +-+ +-+ +--+ |   |            +-+  |   |----+Incrémental|
//!|             Direction +----------> | | | | |  | |   |            +-+  |   |----+           |
//!|                       |          | +-+ +-+ +--+ |   |            | |  |   ||   |           |
//!|             Pwm       +---------->              +---+            | |  |   ||   +-+----+----+
//!|                       |          +--------------+   +------------+ +--+   ++     |    |
//!|                       |                                                          |    |
//!|  Entrée codeur 1      <----------------------------------------------------------+    |
//!|                       |                                                               |
//!|  Entrée codeur 2      <---------------------------------------------------------------+
//!|                       |
//!+-----------------------+
//! ```

mod odometry;
mod motor;
mod pid;

pub use self::motor::*;

use self::odometry::Odometry;
use self::pid::*;
use crate::units::MilliMeter;

use core::f32;
#[allow(unused_imports)]
use libm::F32Ext;

use embedded_hal::Qei;
use qei::QeiManager;

/// Les coordonnées x,y d'un point sur la table
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Coord {
    /// La composante longeur (x)
    pub x: MilliMeter,
    /// La composante largeur (y)
    pub y: MilliMeter,
}

/// Le module central de la navigation, qui permet de controller le robot avec les unités du monde
/// physique, et d'avoir un retour sur la position du robot. Il contient:
/// * un PID basé sur les ticks de roue codeuse
/// * les informations nécessaires pour passer du monde des ticks de roue codeuses au monde physique
/// * les qei gauche et droite correspondant aux deux roues codeuses
/// * la commande à appliquer aux moteurs gauche et droit
pub struct RealWorldPid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
{
    internal_pid: Pid,
    odometry: Odometry,
    params: PIDParameters,
    qei: (QeiManager<L>, QeiManager<R>),
    command: (Command, Command),
}

/// Les paramètres d'un PID
#[derive(Debug, Default, Copy, Clone)]
pub struct PIDParameters {
    /// Le rayon d'une roue codeuse
    pub coder_radius: MilliMeter,
    /// Le nombre de ticks d'une roue codeuse
    pub ticks_per_turn: u16,
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

impl<L, R> core::fmt::Debug for RealWorldPid<L, R>
    where
        L: Qei<Count = u16>,
        R: Qei<Count = u16>,
{
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        write!(
            f,
            "Pid {{ left: {}, right: {} }}",
            self.qei.0.count(),
            self.qei.1.count()
        )
    }
}

impl<L, R> RealWorldPid<L, R>
    where
        L: Qei<Count = u16>,
        R: Qei<Count = u16>,
{

    /// Crée un nouveau PID à partir de :
    /// *  2 struct de `embedded_hal` wrappés dans des `QeiManager`représentant les encodeurs quadratiques gauche et droite
    /// * les coefficients de l'asservissement,
    /// * la valeur maximale de la consigne en sortie,
    /// * les valeurs physiques du robot :
    ///     * distance interaxe en mm
    ///     * rayon d'une roue codeuse en mm
    pub fn new(
        qei_left: QeiManager<L>,
        qei_right: QeiManager<R>,
        params: &PIDParameters,
    ) -> Self {
        RealWorldPid {
            internal_pid: Pid::new(
                params.pos_kp,
                params.pos_kd,
                params.orient_kp,
                params.orient_kd,
                params.max_output,
            ),
            odometry: Odometry::new(),
            params: params.clone(),
            qei: (qei_left, qei_right),
            command: (Command::Front(0), Command::Front(0)),
        }
    }

    /// Mets à jour le PID et la position du robot
    pub fn update(&mut self) {
        self.qei.0.sample_unwrap();
        self.qei.1.sample_unwrap();
        let (left_ticks, right_ticks) = (self.qei.0.count(), self.qei.1.count());
        self.command = self.internal_pid.update(left_ticks, right_ticks);
        self.odometry
            .update(left_ticks, right_ticks, &mut self.params);
    }

    /// Renvoie la commande courante
    pub fn get_command(&self) -> (Command, Command) {
        self.command
    }

    /// Renvoie la position
    pub fn get_position(&self) -> Coord {
        self.odometry.get_position()
    }

    /// Renvoie les ticks comptés par les roues codeuses
    pub fn get_qei_ticks(&self) -> (i64, i64) {
        (self.qei.0.count(), self.qei.1.count())
    }

    /// Ordonne au robot d'avancer de `distance`
    pub fn forward(&mut self, distance: MilliMeter) {
        let distance_per_wheel_turn =
            self.params.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;
        let nb_wheel_turn = distance.as_millimeters() as f32 / distance_per_wheel_turn;
        let ticks = self.params.ticks_per_turn as f32 * nb_wheel_turn;
        self.internal_pid
            .increment_position_goal(ticks.round() as i64);
    }

    /// Ordonne au robot de reculer de `distance`
    pub fn backward(&mut self, distance: MilliMeter) {
        let distance_per_wheel_turn =
            self.params.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;
        let nb_wheel_turn = distance.as_millimeters() as f32 / distance_per_wheel_turn;
        let ticks = self.params.ticks_per_turn as f32 * nb_wheel_turn;
        self.internal_pid.decrement_position_goal(ticks as i64);
    }

    /// Ordonne au robot de tourner de `angle` (en milliradians)
    pub fn rotate(&mut self, angle: i64) {
        let turn_distance =
            angle as f32 * self.params.inter_axial_length.as_millimeters() as f32 * 0.001;
        let distance_per_wheel_turn =
            self.params.coder_radius.as_millimeters() as f32 * 2.0 * core::f32::consts::PI;
        let nb_wheel_turn = turn_distance / distance_per_wheel_turn;
        let ticks = self.params.ticks_per_turn as f32 * nb_wheel_turn;
        self.internal_pid.increment_orientation_goal(ticks as i64);
    }
}

#[cfg(test)]
mod test {
    use qei::QeiManager;

    use super::{PIDParameters, RealWorldPid};
    use super::motor::test::DummyMotor;
    use crate::units::MilliMeter;

    #[test]
    fn real_world_pid_forward() {
        let pid_parameters = PIDParameters {
            coder_radius: MilliMeter(30),
            ticks_per_turn: 1024,
            inter_axial_length: MilliMeter(300),
            pos_kp: 1.0,
            pos_kd: 1.0,
            orient_kp: 1.0,
            orient_kd: 1.0,
            max_output: 100,
        };

        let motor_left = DummyMotor::new();
        let motor_right = DummyMotor::new();
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = RealWorldPid::new(qei_left, qei_right, &pid_parameters);

        pid.forward(MilliMeter(100));

        let (goall, goalr) = pid.internal_pid.get_qei_goal();

        assert!((goall - 543).abs() <= 1, "{} should be {}", goall, 543);
        assert!((goalr - 543).abs() <= 1, "{} should be {}", goalr, 543);
    }

    #[test]
    fn real_world_pid_rotation() {
        let pid_parameters = PIDParameters {
            coder_radius: MilliMeter(30),
            ticks_per_turn: 1024,
            inter_axial_length: MilliMeter(300),
            pos_kp: 1.0,
            pos_kd: 1.0,
            orient_kp: 1.0,
            orient_kd: 1.0,
            max_output: 100,
        };

        let motor_left = DummyMotor::new();
        let motor_right = DummyMotor::new();
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = RealWorldPid::new(qei_left, qei_right, &pid_parameters);

        pid.rotate(785); // PI / 4 en milliradians

        let (goall, goalr) = pid.internal_pid.get_qei_goal();

        assert!((goall + 640).abs() <= 1, "{} should be {}", goall, -640);
        assert!((goalr - 640).abs() <= 1, "{} should be {}", goalr, 640);
    }
    
    #[test]
    fn test_full_session() {}
}
