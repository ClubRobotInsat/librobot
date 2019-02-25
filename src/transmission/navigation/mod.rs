use crate::transmission::Jsonizable;
use heapless::{ArrayLength, String};
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

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
    command: NavigationCommand,
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
pub enum NavigationCommand {
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

impl Jsonizable for NavigationFrame {
    /// Construit une trame a partir d'un flux de donnees json.
    fn from_json_slice(slice: &[u8]) -> Result<Self, DError> {
        from_slice(slice)
    }

    /// Construit une chaine de caractère en json à partir de cette trame
    fn to_string<B>(&self) -> Result<String<B>, SError>
    where
        B: ArrayLength<u8>,
    {
        to_string(self)
    }
}
