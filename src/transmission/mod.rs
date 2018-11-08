//! Ensemble des types, fonctions et traits qui permettent la communication electronique <-> informatique.
//!
//! Le code qui permet de transformer des octets en structures et inversemment est écrit en C.
//! Pour chaque type de messages, il y a une structure C (que vous ne voyez pas car elle est cachée)
//! et une structure Rust qui correspond exactement à celle en C.
//!
//! Au niveau de la librairie vous ne voyez ni l'une, ni l'autre car on expose une structure encore
//! Rust plus idiomatique.Par exemple, pour transmettre des servomoteurs l'informatique enverra des
//! messages contenant des [servo::ServoGroup].
//!
//! # Forme d'une Frame
//!
//! ***TODO***
//!
//! # Exemple
//!
//! Voici un exemple de communication PC-Elec :
//! ```c++
//! +----------------+           +--------------------+
//! |                |           |                    |
//! |  Raspberry PI  |           |  Microcontrolleur  |
//! |                |           |                    |
//! +--------+-------+           +----------+---------+
//!          |                              |
//!          |  AC DC AB BA 01 05 06 00     |
//!          | +--------------------------> |
//!          |                              |
//!          |  AC DC AB BB 01 00 00        |
//!          | <--------------------------+ |
//!          |                              |
//!          |  AC DC AB BA 05 46 02 11 77  |
//!          | <--------------------------+ |
//!          |                              |
//!          v(t)                           v(t)
//! ```
//!
//! Voici comment lire des messages et renvoyer des informations vers l'informatique pour les
//! servomoteurs.
//!
//! ```c++
//!  +--------------------------+                  +---------------------+
//! |                          |                  |                     |
//! |       Conversion en      |                  | Conversion en Frame |
//! |          Octets          +----------------->+                     |
//! |                          |    ServoGroup    |    Frame::new()     |
//! | ServoGroup::into_bytes() |                  |                     |
//! |                          |                  +------+--------------+
//! +--------------------------+                         |
//!                                                      |
//!                                                      |
//!                                        Connexion     |Donnée
//!                                          Série       |à émettre
//!                                            +         |
//!                                            |         |
//!                                            v         v
//!                      +----------------+RX      TX+---+------------+-------------------+
//!                      |                +-<-<-<-<-<+Emission        |  Périphérique     |
//!                      | Raspberry PI   |          +----------------+      UART         |
//!                      |                +->->->->->+Réception       |                   |
//!                      +----------------+TX      RX+---+------------+-------------------+
//!                                                      |
//!                                                      | Donnée
//!                                                      | Reçue
//!                                                      |
//!                                                      v
//!+---------------------+                  +------------+-----------+
//!|                     |                  |                        |
//!|   Conversion de     |        Frame     | Extraction des données |
//!|    la Frame en      +<-----------------+  FrameReader.step()    |
//!|     ServoGroup      |                  |                        |
//!|                     |                  +------------------------+
//!|  ServoGroup::new()  |
//!|                     |
//!+---------+-----------+
//!          |
//!          |
//!          |ServoGroup
//!          |
//!          v
//!     +----+----+
//!     |         |
//!     | Gestion |
//!     |   du    |
//!     | Message |
//!     |         |
//!     +---------+
//!```

use arrayvec::ArrayVec;

#[macro_use]
mod frame;
mod ffi;
mod frame_reader;
pub mod servo;

pub use self::frame::*;
pub use self::frame_reader::*;
pub use self::ffi::ErrorParsing;

/// Taille maximale du message véhiculé par la frame
pub const FRAME_MAX_SIZE: usize = 256 /* - 6*/;
/// Un message est un tableau de 256 octets.
pub type Message = ArrayVec<[u8; FRAME_MAX_SIZE]>;
