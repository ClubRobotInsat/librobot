//! Ce module contient tout le code qui permets au robot de se déplacer sur la table.
//!
//! Pour rappel, la carte est câblée ainsi :
//!```ignore
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

/// Le système d'unité que l'on va utiliser pour le déplacement
pub mod ms {
    make_units! {
        MS;
        ONE: Unitless;

        base {
            MM: MilliMeter, "mm", Length;
            MICROS: MicroSecond, "µs", Time;
            S: Second, "s", Time;
        }

        derived {
            MPS: MilliMeterPerSecond = (MilliMeter / Second), Velocity;
            HZ: Hertz = (Unitless / Second), Frequency;
        }

        constants {
            PI: Unitless = consts::PI;
        }

        fmt = true;
    }
    pub use self::f64consts::*;
}

use self::ms::MilliMeter;

/// Les coordonnées x,y d'un point sur la table
#[derive(Debug, Copy, Clone)]
pub struct Coord {
    /// La composante longeur (x)
    pub x: MilliMeter<i64>,
    /// La composante largeur (x)
    pub y: MilliMeter<i64>,
}
