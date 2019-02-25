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
mod pid;

use self::odometry::Odometry;
pub use self::pid::*;

use crate::units::MilliMeter;

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

/// Toutes les informations nécessaires sur les roues codeuses
#[derive(Debug)]
pub struct RobotConstants {
    /// rayon des roues codeuses
    pub coder_radius: MilliMeter,
    /// longueur entre les deux roues
    pub inter_axial_length: MilliMeter,
}

/// PID + ODOMETRY
pub struct Pid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
{
    pid: RealWorldPid,
    odometry: Odometry,
    constants: RobotConstants,
    qei: (QeiManager<L>, QeiManager<R>),
    command: (Command, Command),
}

impl<L, R> core::fmt::Debug for Pid<L, R>
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

impl<L, R> Pid<L, R>
where
    L: Qei<Count = u16>,
    R: Qei<Count = u16>,
{
    /// Crée un nouveau PID
    pub fn new(
        qei_left: QeiManager<L>,
        qei_right: QeiManager<R>,
        params: &PIDParameters,
        constants: RobotConstants,
    ) -> Self {
        Pid {
            pid: RealWorldPid::new(params),
            odometry: Odometry::new(),
            constants,
            qei: (qei_left, qei_right),
            command: (Command::Front(0), Command::Front(0)),
        }
    }

    /// Mets à jour le PID et la position du robot
    pub fn update(&mut self) {
        self.qei.0.sample_unwrap();
        self.qei.1.sample_unwrap();
        let (left_ticks, right_ticks) = (self.qei.0.count(), self.qei.1.count());
        self.command = self.pid.update(left_ticks, right_ticks);
        self.odometry
            .update(left_ticks, right_ticks, &mut self.constants);
    }

    /// Renvoie la commande courante
    pub fn get_command(&self) -> (Command, Command) {
        self.command
    }

    /// Renvoie la position
    pub fn get_position(&self) -> Coord {
        self.odometry.get_position()
    }

    /// Fais avancer le robot
    /// FIXME
    pub fn forward(&mut self, distance: MilliMeter) {
        self.pid.forward(distance);
    }

    /// Renvoie les ticks comptés par les roues codeuses
    pub fn get_qei_ticks(&self) -> (i64, i64) {
        (self.qei.0.count(), self.qei.1.count())
    }
}

#[cfg(test)]
mod test {

    #[test]
    fn test_full_session() {}
}
