//! Décrit l'API pour interagir avec la carte IO

use crate::transmission::Jsonizable;
use heapless::{ArrayLength, String};
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

/// L'état d'un interrupteur : en attente d'activation, ou activé
#[derive(Debug, Copy, Clone, Deserialize, Serialize)]
pub enum TriggerState {
    /// En attente d'activation
    Triggered,
    /// Activé
    Waiting,
}

/// L'état d'un port IO : On ou Off
#[derive(Debug, Copy, Clone, Deserialize, Serialize)]
pub enum IOState {
    /// Le port est activé (état haut)
    On,
    /// Le port est éteinds (état bas)
    Off,
}

/// L'état du robot d'un point de vue IO
#[derive(Debug, Copy, Clone, Deserialize, Serialize)]
pub struct IO {
    /// L'état des pompes (elles sont sur le même pin, même si il y en a 2)
    pumps: IOState,
    /// L'état de la tirette
    tirette: TriggerState,
    /// L'état des vannes
    valves: [IOState; 8],
}

impl Jsonizable for IO {
    /// Désérialisation d'un JSON en `Servo`
    fn from_json_slice(slice: &[u8]) -> Result<Self, DError> {
        from_slice(slice)
    }

    /// Sérialisation d'un `Servo` en JSON
    fn to_string<B>(&self) -> Result<String<B>, SError>
    where
        B: ArrayLength<u8>,
    {
        to_string(self)
    }
}
