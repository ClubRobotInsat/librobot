//! Ce module contient tout le code qui permets au robot de se déplacer sur la table.
//!
//! Pour rappel, la carte est câblée ainsi :
//!
//!>                                                     ^----+Sortie du pont H
//!>                                                     |
//!>                                                     |
//!>                                    +--------------+ | +------------+ +--+   ++
//!> +-----------------------+          | Pont H       +-v-+ Moteur DC  | |  |   ||   +-----------+
//!> |    Carte de commande  |          |              |   |            | |  |   ||   |Codeur     |
//!> |                       |          | +-+ +-+ +--+ |   |            +-+  |   |----+Incrémental|
//!> |             Direction +----------> | | | | |  | |   |            +-+  |   |----+           |
//!> |                       |          | +-+ +-+ +--+ |   |            | |  |   ||   |           |
//!> |             Pwm       +---------->              +---+            | |  |   ||   +-+----+----+
//!> |                       |          +--------------+   +------------+ +--+   ++     |    |
//!> |                       |                                                          |    |
//!> |  Entrée codeur 1      <----------------------------------------------------------+    |
//!> |                       |                                                               |
//!> |  Entrée codeur 2      <---------------------------------------------------------------+
//!> |                       |
//!> +-----------------------+
//!>

pub mod pid;

use units::MilliMeter;

/// Les coordonnées x,y d'un point sur la table
#[derive(Debug, Copy, Clone)]
pub struct Coord {
    /// La composante longeur (x)
    pub x: MilliMeter,
    /// La composante largeur (x)
    pub y: MilliMeter,
}

impl Coord {
    /// Crées des coordonnées à partir d'un nombre de tick
    pub fn from_tick_count(left : i64, right : i64, inter_axial : MilliMeter, coder_radius : MilliMeter) -> Self {
        unimplemented!()
        /*
        Coord {
            x : 0,
            y : 0
        }
        */
    }
}