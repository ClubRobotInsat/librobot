//! Décrit l'API pour interagir avec la carte IO

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
    vannes: [IOState; 8],
}
