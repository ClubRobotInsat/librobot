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

mod motor;
mod odometry;
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
    /// Le rayon d'une roue codeuse en mm
    pub coder_radius: f32,
    /// Coefficient de correction de la roue codeuse gauche, notamment
    /// pour pouvoir supporter le décompte en sens inverse
    pub left_wheel_coef: f32,
    /// Coefficient de correction de la roue codeuse droite, pour corriger
    /// le sens et l'écart de diamètre entre les deux roues.
    pub right_wheel_coef: f32,
    /// Le nombre de ticks d'une roue codeuse
    pub ticks_per_turn: u16,
    /// La distance entre les roues codeuses en mm
    pub inter_axial_length: f32,
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
    pub fn new(qei_left: QeiManager<L>, qei_right: QeiManager<R>, params: &PIDParameters) -> Self {
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
        let (left_dist, right_dist) = self.params.ticks_to_distance(left_ticks, right_ticks);
        self.command = self.internal_pid.update(left_dist, right_dist);
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

    /// Renvoie l'angle en milliradians
    pub fn get_angle(&self) -> i64 {
        self.odometry.get_angle()
    }

    /// Renvoie les ticks comptés par les roues codeuses
    pub fn get_qei_ticks(&self) -> (i64, i64) {
        (self.qei.0.count(), self.qei.1.count())
    }

    /// Ordonne au robot d'avancer de `distance` (en mm)
    pub fn forward(&mut self, distance: f32) {
        self.internal_pid.increment_goal(distance, distance);
    }

    /// Ordonne au robot de reculer de `distance` (en mm)
    pub fn backward(&mut self, distance: f32) {
        self.internal_pid.increment_goal(-distance, -distance);
    }

    /// Ordonne au robot de tourner de `angle` (en milliradians)
    pub fn rotate(&mut self, angle: f32) {
        let turn_distance = angle * self.params.inter_axial_length * (0.001 / 2.);
        self.internal_pid
            .increment_goal(-turn_distance, turn_distance);
    }

    /// Ordonne au robot de rester là où il est actuellement
    pub fn stop(&mut self) {
        let (left_ticks, right_ticks) = self.get_qei_ticks();
        let (left_dist, right_dist) = self.params.ticks_to_distance(left_ticks, right_ticks);
        self.internal_pid.set_goal(left_dist, right_dist);
    }

    /// Retourne `true` si le pid a atteind sa consigne en position et angle
    pub fn is_goal_reached(&self, lin_accuracy: f32, ang_accuracy: f32) -> bool {
        let (left_ticks, right_ticks) = self.get_qei_ticks();
        let (left_dist, right_dist) = self.params.ticks_to_distance(left_ticks, right_ticks);
        let (left_goal, right_goal) = self.internal_pid.get_goal();
        let lin_gap = (left_dist + right_dist - left_goal - right_goal) / 2.0;
        let ang_gap =
            (left_dist - right_dist - left_goal + right_goal) / self.params.inter_axial_length;
        lin_gap.abs() < lin_accuracy && ang_gap.abs() < ang_accuracy / 1000.0
    }
}

impl PIDParameters {
    /// Convertit les ticks des QEI en distance parcourue par les roues codeuses (en mm)
    pub fn ticks_to_distance(&self, left_ticks: i64, right_ticks: i64) -> (f32, f32) {
        let distance_per_wheel_turn = self.coder_radius * 2.0 * core::f32::consts::PI;

        (
            left_ticks as f32 * distance_per_wheel_turn * self.left_wheel_coef
                / self.ticks_per_turn as f32,
            right_ticks as f32 * distance_per_wheel_turn * self.right_wheel_coef
                / self.ticks_per_turn as f32,
        )
    }

    /// Convertit la distance parcourue en mm par chaque roue codeuse, en nombre de ticks
    /// observé par chaque QEI.
    pub(crate) fn distancef_to_ticks(&self, left_distance: f32, right_distance: f32) -> (i64, i64) {
        let distance_per_wheel_turn = self.coder_radius * 2.0 * core::f32::consts::PI;

        (
            (left_distance * self.ticks_per_turn as f32
                / (distance_per_wheel_turn * self.left_wheel_coef)) as i64,
            (right_distance * self.ticks_per_turn as f32
                / (distance_per_wheel_turn * self.right_wheel_coef)) as i64,
        )
    }

    /// Convertit la distance parcourue par chaque roue codeuse, en nombre de ticks
    /// observé par chaque QEI.
    pub fn distance_to_ticks(
        &self,
        left_distance: MilliMeter,
        right_distance: MilliMeter,
    ) -> (i64, i64) {
        self.distancef_to_ticks(
            left_distance.as_millimeters() as f32,
            right_distance.as_millimeters() as f32,
        )
    }
}

#[cfg(test)]
mod test {
    use qei::QeiManager;

    use super::motor::test::DummyMotor;
    use super::{PIDParameters, RealWorldPid};
    use crate::navigation::Command;
    use crate::units::MilliMeter;

    #[test]
    fn test_ticks_to_distance() {
        let pid_parameters = PIDParameters {
            coder_radius: 30.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 0.5,
            ticks_per_turn: 1024,
            inter_axial_length: 300.0,
            pos_kp: 1.0,
            pos_kd: 1.0,
            orient_kp: 1.0,
            orient_kd: 1.0,
            max_output: 100,
        };

        let (left_dist, right_dist) = pid_parameters.ticks_to_distance(1280, 1280);
        assert!(
            (left_dist - 235.6).abs() < 0.2,
            "{} should be 235.6",
            left_dist
        );
        assert!(
            (right_dist - 117.8).abs() < 0.2,
            "{} should be 117.8",
            right_dist
        );
    }

    #[test]
    fn test_distance_to_ticks() {
        let pid_parameters = PIDParameters {
            coder_radius: 30.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 0.5,
            ticks_per_turn: 1024,
            inter_axial_length: 300.0,
            pos_kp: 1.0,
            pos_kd: 1.0,
            orient_kp: 1.0,
            orient_kd: 1.0,
            max_output: 100,
        };
        let (left_ticks, right_ticks) =
            pid_parameters.distance_to_ticks(MilliMeter(235), MilliMeter(235));
        assert!(
            (left_ticks - 1276).abs() <= 1,
            "{} should be 1276",
            left_ticks
        );
        assert!(
            (right_ticks - 2552).abs() <= 1,
            "{} should be 1276",
            right_ticks
        );
    }

    #[test]
    fn test_goal_reached() {
        let pid_parameters = PIDParameters {
            coder_radius: 30.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 300.0,
            pos_kp: 1.0,
            pos_kd: 1.0,
            orient_kp: 1.0,
            orient_kd: 1.0,
            max_output: 100,
        };

        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = RealWorldPid::new(qei_left, qei_right, &pid_parameters);

        pid.internal_pid.set_goal(10.0, 10.0);

        assert!(pid.is_goal_reached(11.0, 0.1));
        assert!(!pid.is_goal_reached(4.0, 0.1));

        pid.internal_pid.set_goal(10.0, -10.0);
        motor_left.set_position(49); // 9 mm
        motor_right.set_position(-65); // 12 mm

        pid.qei.0.sample_unwrap();
        pid.qei.1.sample_unwrap();
        let (left_ticks, right_ticks) = pid.get_qei_ticks();
        let (left_dist, right_dist) = pid_parameters.ticks_to_distance(left_ticks, right_ticks);

        assert!(
            pid.is_goal_reached(2.0, 5.0),
            "success with {}, {}",
            left_dist,
            right_dist
        );
        assert!(
            !pid.is_goal_reached(2.0, 2.0),
            "angular fails with {}, {}",
            left_dist,
            right_dist
        );
        assert!(
            !pid.is_goal_reached(0.5, 5.0),
            "linear fails with {}, {}",
            left_dist,
            right_dist
        );
    }

    /*
    #[test]
    fn real_world_pid_forward() {
        let pid_parameters = PIDParameters {
            coder_radius: 30.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: 0.5,
            ticks_per_turn: 1024,
            inter_axial_length: 300.0,
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
    */

    #[test]
    fn real_world_pid_rotation() {
        let pid_parameters = PIDParameters {
            coder_radius: 30.0,
            left_wheel_coef: 1.0,
            right_wheel_coef: -1.0,
            ticks_per_turn: 1024,
            inter_axial_length: 300.0,
            pos_kp: 1.0,
            pos_kd: 0.0,
            orient_kp: 1.0,
            orient_kd: 0.0,
            max_output: 100,
        };

        let mut motor_left = DummyMotor::new();
        let mut motor_right = DummyMotor::new();
        let qei_left = QeiManager::new(motor_left.clone());
        let qei_right = QeiManager::new(motor_right.clone());
        let mut pid = RealWorldPid::new(qei_left, qei_right, &pid_parameters);

        pid.rotate(785.0); // PI / 4 en milliradians

        let (goall, goalr) = pid.internal_pid.get_goal();

        assert!((goall + 117.0).abs() <= 1.0, "{} should be {}", goall, -117);
        assert!((goalr - 117.0).abs() <= 1.0, "{} should be {}", goalr, 117);

        pid.update();
        let (cmd_left, cmd_right) = pid.get_command();

        match cmd_left {
            Command::Back(_) => {}
            _ => panic!("cmd_left should be going backward."),
        }
        match cmd_right {
            Command::Front(_) => {}
            _ => panic!("cmd_right should be going forward."),
        }

        motor_left.set_position(-1024);
        motor_right.set_position(-1024);

        pid.update();
        let (cmd_left2, cmd_right2) = pid.get_command();

        match cmd_left2 {
            Command::Front(_) => {}
            _ => panic!("cmd_left2 should be going forward."),
        }
        match cmd_right2 {
            Command::Back(_) => {}
            _ => panic!("cmd_right2 should be going backward."),
        }
    }

    #[test]
    fn test_full_session() {}
}
