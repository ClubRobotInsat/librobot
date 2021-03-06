//! Décrit l'API pour interagir avec la carte déplacement

mod params;

use crate::transmission::Jsonizable;
use heapless::{ArrayLength, String};
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

pub use self::params::NavigationParametersFrame;

/// Trame contenant les informations echangees entre l'info et l'elec.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default, Deserialize, Serialize)]
pub struct NavigationFrame {
    // Variables d'état écrites par l'elec
    /// position x du robot en dixieme de millimetres
    pub x: i32,
    /// position y du robot en dixieme de millimetres
    pub y: i32,
    /// angle du robot en centaines de microradians
    pub angle: i32,
    /// distance parcourue par la roue gauche en millimètres
    pub left_dist: i32,
    /// distance parcourue par la roue droite en millimètres
    pub right_dist: i32,
    /// vrai si le robot ne peut pas avancer
    pub blocked: bool,
    /// vrai si le robot a fini d'executer la commande
    pub moving_done: bool,

    // Variables d'état écrites par l'info
    /// vrai si l'asservissement longitudinal est operationnel
    pub asserv_lin: bool,
    /// vrai si l'asservissement angulaire est opérationnel
    pub asserv_ang: bool,
    /// eclairage des LEDs du module (si elles sont presentes)
    pub led: bool,
    /// si vrai, l'info peut fixer (x, y, angle)
    pub reset: bool,

    /// vitesse longitudinale max du robot en mm/s
    pub max_lin_speed: u16,
    /// vitesse angulaire max du robot en milliradian/s
    pub max_ang_speed: u16,
    /// précision longitudinale du robot (à partir de laquelle on considère
    /// qu'une commande de déplacement longitudinal a été réalisée)
    /// en dixième de millimetre
    pub lin_accuracy: u16,
    /// précision angulaire du robot (à partir de laquelle on considère qu'une
    /// commande de déplacement angulaire a été réalisée)
    /// en dixième de milliradian
    pub ang_accuracy: u16,

    // Commande actuelle
    /// commande à effectuer
    pub command: NavigationCommand,
    /// argument 1 de la commande
    pub args_cmd1: u16,
    /// argument 2 de la commande
    pub args_cmd2: u16,
    /// numéro de la commande en cours. Si on reçoit une commande
    /// avec un numéro plus grand, on l'execute en priorité
    pub counter: u16,
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

impl Default for NavigationCommand {
    fn default() -> Self {
        NavigationCommand::DoNothing
    }
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
    use heapless::consts::U512;
    use heapless::String;
    type N = U512;

    #[test]
    fn ser_deser_navigation_forward() {
        let nav = NavigationFrame {
            x: 0,
            y: 0,
            angle: 0,
            left_dist: 0,
            right_dist: 0,
            blocked: false,
            moving_done: false,

            asserv_lin: true,
            asserv_ang: true,
            led: true,
            reset: true,
            max_lin_speed: 1000,
            max_ang_speed: 3000,
            lin_accuracy: 40,
            ang_accuracy: 20,

            command: NavigationCommand::GoForward,
            args_cmd1: 500,
            args_cmd2: 0,
            counter: 1,
        };
        let strd: String<N> = nav.to_string().unwrap();
        let data =
            "{\"angle\":0,\"args_cmd1\":500,\"args_cmd2\":0,\"blocked\":false,\"command\":\"GoForward\",\
            \"counter\":1,\"led\":true,\"moving_done\":false,\"reset\":true,\"x\":0,\"y\":0,\"max_lin_speed\":1000,\"max_ang_speed\":3000,\
            \"lin_accuracy\":40,\"ang_accuracy\":20,\"asserv_lin\":true,\"asserv_ang\":true,\"left_dist\":0,\"right_dist\":0}";
        let nav2 = NavigationFrame::from_json_slice(strd.as_bytes()).unwrap();
        assert_eq!(nav, nav2);
        let nav3 = NavigationFrame::from_json_slice(data.as_bytes()).unwrap();
        assert_eq!(nav, nav3);
    }

}
