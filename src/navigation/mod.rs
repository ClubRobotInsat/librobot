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

mod pid;

pub use self::pid::*;
use units::MilliMeter;
use serde_json_core::de::{from_slice, Error as DError};

/// Les coordonnées x,y d'un point sur la table
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Coord {
    /// La composante longeur (x)
    pub x: MilliMeter,
    /// La composante largeur (x)
    pub y: MilliMeter,
}

impl Coord {
    /// Crées des coordonnées à partir d'un nombre de tick
    pub fn from_tick_count(
        _left: i64,
        _right: i64,
        _inter_axial: MilliMeter,
        _coder_radius: MilliMeter,
    ) -> Self {
        unimplemented!()
        /*
        Coord {
            x : 0,
            y : 0
        }
        */
    }
}

/// Trame contenant les informations echangees entre l'info et l'elec.
#[derive(Debug, Default, Copy, Clone, PartialEq, Eq, Deserialize, Serialize)]
pub struct NavigationFrame {
    /// position x du robot en dixieme de millimetres
    x:u16,
    /// position y du robot en dixieme de millimetres
    y:u16,
    /// angle du robot en centaines de microradians
    angle:u16,
    /// vrai si le robot ne peut pas avancer
    blocked:bool,
    /// vrai si l'asservissement est operationnel
    asserv_on_off:bool,
    /// eclairage des LEDs du module (si elles sont presentes)
    led:bool,
    /// si vrai, l'info peut fixer (x, y,
    reset:bool
}

impl NavigationFrame {
    /// Construit une trame a partir d'un flux de donnees json.
    pub fn from_json_slice(slice: &[u8]) -> Result<Self, DError> {
        from_slice(slice)
    }
}
