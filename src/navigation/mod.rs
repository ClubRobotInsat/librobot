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
use heapless::{ArrayLength, String};

use embedded_hal::Qei;
use qei::QeiManager;
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

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

/// Trame contenant les informations echangees entre l'info et l'elec.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct NavigationFrame {
    /// position x du robot en dixieme de millimetres
    x: u16,
    /// position y du robot en dixieme de millimetres
    y: u16,
    /// angle du robot en centaines de microradians
    angle: u16,
    /// vrai si le robot ne peut pas avancer
    blocked: bool,
    /// vrai si l'asservissement est operationnel
    asserv_on_off: bool,
    /// eclairage des LEDs du module (si elles sont presentes)
    led: bool,
    /// si vrai, l'info peut fixer (x, y, angle)
    reset: bool,
    /// commande à effectuer
    command: MacroCommand,
    /// argument 1 de la commande
    args_cmd1: u16,
    /// argument 2 de la commande
    args_cmd2: u16,
    /// numéro de la commande en cours. Si on reçoit une commande
    /// avec un numéro plus grand, on l'execute en priorité
    counter: u16,
    /// vrai si le robot a fini d'executer la commande
    moving_done: bool,
}

/// Les differentes commandes que le déplacement peut effectuer
#[derive(Debug, PartialEq, Copy, Clone, Eq, Serialize, Deserialize)]
pub enum MacroCommand {
    /// avancer.
    /// Arguments : distance, _
    GoForward,
    /// reculer.
    /// Arguments : distance, _
    GoBackward,
    /// tourner d'un certain angle relativement à l'angle actuel du robot.
    /// Arguments : angle, _
    TurnRelative,
    /// tourner de manière à se positionner à l'angle voulu.
    /// Arguments : angle, _
    TurnAbsolute,
    /// ne rien faire
    DoNothing,
    /// s'arrêter d'urgence
    EmergencyStop,
    /// s'arrêter, mais pas d'urgence
    Stop,
}

impl NavigationFrame {
    /// Construit une trame a partir d'un flux de donnees json.
    pub fn from_json_slice(slice: &[u8]) -> Result<Self, DError> {
        from_slice(slice)
    }

    /// Construit une chaine de caractère en json à partir de cette trame
    pub fn to_string<B>(&self) -> Result<String<B>, SError>
    where
        B: ArrayLength<u8>,
    {
        to_string(self)
    }
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
}

#[cfg(test)]
mod test {}
