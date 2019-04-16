//! Décrit l'API pour interagir avec la carte déplacement

use crate::transmission::Jsonizable;
use heapless::{ArrayLength, String};
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

/// Trame contenant les informations echangees entre l'info et l'elec.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct NavigationFrame {
    /// position x du robot en dixieme de millimetres
    pub x: u16,
    /// position y du robot en dixieme de millimetres
    pub y: u16,
    /// angle du robot en centaines de microradians
    pub angle: u16,
    /// vrai si le robot ne peut pas avancer
    pub blocked: bool,
    /// vrai si l'asservissement est operationnel
    pub asserv_on_off: bool,
    /// eclairage des LEDs du module (si elles sont presentes)
    pub led: bool,
    /// si vrai, l'info peut fixer (x, y, angle)
    pub reset: bool,
    /// commande à effectuer
    pub command: NavigationCommand,
    /// argument 1 de la commande
    pub args_cmd1: u16,
    /// argument 2 de la commande
    pub args_cmd2: u16,
    /// numéro de la commande en cours. Si on reçoit une commande
    /// avec un numéro plus grand, on l'execute en priorité
    pub counter: u16,
    /// vrai si le robot a fini d'executer la commande
    pub moving_done: bool,
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

#[cfg(test)]
mod test {
    use super::{NavigationCommand, NavigationFrame};
    use crate::transmission::Jsonizable;
    use heapless::consts::U256;
    use heapless::String;
    type N = U256;

    #[test]
    fn ser_deser_navigation_forward() {
        let nav = NavigationFrame {
            angle: 0,
            args_cmd1: 500,
            args_cmd2: 0,
            asserv_on_off: true,
            blocked: false,
            command: NavigationCommand::GoForward,
            counter: 1,
            led: true,
            moving_done: false,
            reset: true,
            x: 0,
            y: 0,
        };
        let strd: String<N> = nav.to_string().unwrap();
        let data =
            "{\"angle\":0,\"args_cmd1\":500,\"args_cmd2\":0,\"asserv_on_off\":true,\"blocked\":false,\"command\":\"GoForward\",\"counter\":1,\"led\":true,\"moving_done\":false,\"reset\":true,\"x\":0,\"y\":0}";
        let nav2 = NavigationFrame::from_json_slice(strd.as_bytes()).unwrap();
        assert_eq!(nav, nav2);
        let nav3 = NavigationFrame::from_json_slice(data.as_bytes()).unwrap();
        assert_eq!(nav, nav3);
    }

}
