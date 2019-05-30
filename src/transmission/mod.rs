//! Ensemble des types, fonctions et traits qui permettent la communication electronique <-> informatique.
//!
//! Le code qui permet de transformer des octets en structures et inversemment est écrit en C.
//! Pour chaque type de messages, il y a une structure C (que vous ne voyez pas car elle est cachée)
//! et une structure Rust qui correspond exactement à celle en C.
//!
//! Au niveau de la librairie vous ne voyez ni l'une, ni l'autre car on expose une structure encore
//! Rust plus idiomatique.Par exemple, pour transmettre des servomoteurs l'informatique enverra des
//! messages contenant des [`servo::ServoGroup`].
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

pub mod eth;

pub mod color;
pub mod io;
pub mod navigation;
pub mod servo;

use heapless::{ArrayLength, String};

use serde_json_core::de::Error as DError;
use serde_json_core::ser::Error as SError;

/// Taille maximale du message véhiculé par la frame
pub const FRAME_MAX_SIZE: usize = 256 /* - 6*/;
/// Un message est un tableau de 256 octets.
pub type Message = ArrayVec<[u8; FRAME_MAX_SIZE]>;

/// Regroupe les identifiants des cartes qui sont utilisés pour calculer les ports d'écoute et d'envoi
/// lors de la communication par UPD
pub mod id {
    /// L'ID de la carte déplacement
    pub const ID_NAVIGATION: u16 = 1;

    /// L'ID de la carte servo
    pub const ID_SERVO: u16 = 2;

    /// L'ID de la carte IO (tirette & buzzer)
    pub const ID_IO: u16 = 4;

    /// L'ID de la carte pneumatique
    pub const ID_PNEUMATIC: u16 = 5;

    /// L'ID de la carte couleur
    pub const ID_COLOR: u16 = 6;

    /// L'ID des paramètres de la navigation
    pub const ID_NAVIGATION_PARAMETERS: u16 = 10;

    /// Le port auquel il faut ajouter l'ID pour envoyer des trames à l'informatique
    pub const INFO_LISTENING_PORT: u16 = 5000;

    /// Le port auquel il faut ajouter l'ID pour recevoir des trames de l'informatique
    pub const ELEC_LISTENING_PORT: u16 = 50;
}

/// Le type de message
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum MessageKind {
    /// Commande de servomoteur
    Servo,
    /// Commande de déplacement
    Navigation,
}

impl Into<u8> for MessageKind {
    fn into(self) -> u8 {
        match self {
            MessageKind::Servo => 4,
            MessageKind::Navigation => 5, // TODO : agree into
        }
    }
}

impl MessageKind {
    /// Crée un `MessageKind` depuis un entier
    pub fn from_u8(data: u8) -> Result<MessageKind, ()> {
        match data {
            4 => Ok(MessageKind::Servo),
            5 => Ok(MessageKind::Navigation),
            _ => Err(()),
        }
    }
}

/// Traits utilitaires implémentés par toutes les structures que l'on envoie/récupère du réseau
pub trait Jsonizable
where
    Self: core::marker::Sized,
{
    /// Désérialisation d'un JSON en `Servo`
    fn from_json_slice(slice: &[u8]) -> Result<Self, DError>;

    /// Sérialisation d'un `Servo` en JSON
    fn to_string<B>(&self) -> Result<String<B>, SError>
    where
        B: ArrayLength<u8>;
}
