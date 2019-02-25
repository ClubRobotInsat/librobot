//! Représentation haut-niveau d'un servo-moteur.

use crate::transmission::Jsonizable;
use heapless::{ArrayLength, String};
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

/// Représentation d'un unique servo-moteur
#[derive(Debug, Default, Copy, Clone, Eq, Deserialize, Serialize)]
pub struct Servo {
    // TODO : spécifier les histoires d'ID = 0
    /// Identifiant du servo-moteur.
    pub id: u8,
    /// Position actuelle du servo-moteur.
    pub known_position: u16,
    /// Commande du servo soit en angle soit en vitesse.
    pub control: Control,
    /// Sens de rotation associé à une commande en vitesse.
    /// TODO : on doit remplir de champ pour une commande en position aussi, mais je ne sais pas
    /// comment faire rentrer ce champ dans le `control` : en faisant une enum plus intelligente :
    /// ```txt
    /// pub enum Control {
    ///     Speed {
    ///         rotation: Rotation,
    ///     },
    ///     Position,
    /// }
    /// ```
    /// la lib `serde_json_core` n'est pas capable de désérialiser (elle attend des types primitifs
    /// mais on lui donne une structure complexe à manger, aka `Control`) -- @Terae
    pub rotation: Rotation,
    /// Représente les informations de contrôle associées à la commande `Speed` ou `Position`.
    pub data: u16,
    /// Retourne vrai si le servo-moteur est bloqué
    pub blocked: bool,
    /// Comportement du servo-moteur face à un blocage extérieur.
    pub mode: BlockingMode,
    /// Couleur émise par le servo-moteur.
    pub color: Color,
}

impl Jsonizable for Servo {
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

/// Un ensemble de au plus 8 servos-moteurs
#[derive(PartialEq, Debug, Clone, Serialize, Deserialize)]
pub struct ServoGroup {
    /// Vecteur d'au plus 8 servos-moteurs
    pub servos: Servo,
}

/// Relation d'équivalence partielle pour le module `Servo2019`, utile pour le débug.
impl PartialEq for Servo {
    fn eq(&self, other: &Servo) -> bool {
        self.id == other.id
            && (self.id == 0
                || (self.known_position == other.known_position
                    && self.control == other.control
                    && self.blocked == other.blocked
                    && self.mode == other.mode
                    && self.color == other.color))
    }
}

/// Comportement du servo-moteur lorsqu'il est bloqué.
#[derive(Debug, PartialEq, Copy, Clone, Eq, Serialize, Deserialize)]
pub enum BlockingMode {
    /// Le servo relâche la pression lorsqu'il est bloqué.
    Unblocking = 0,
    /// Le servo maintient un couple pour s'opposer au blocage.
    HoldOnBlock = 1,
}

impl Default for BlockingMode {
    fn default() -> Self {
        BlockingMode::Unblocking
    }
}

/// Représente le sens de rotation du servo moteur lorsqu'il est contrôle en vitesse
#[derive(Debug, PartialEq, Copy, Clone, Eq, Serialize, Deserialize)]
pub enum Rotation {
    /// Rotation trigonométrique, qui est le sens par défaut
    CounterClockwise,
    /// Rotation horaire, ce qui représente une rotation inverse
    Clockwise,
}

impl Default for Rotation {
    fn default() -> Self {
        Rotation::CounterClockwise
    }
}

/// Commande du servo-moteur.
#[derive(Debug, PartialEq, Copy, Clone, Eq, Serialize, Deserialize)]
pub enum Control {
    /// Commande en vitesse.
    Speed,
    /// Commande en position.
    Position,
}

impl Default for Control {
    fn default() -> Self {
        Control::Position
    }
}

/// Couleur émise par le servo-moteur.
#[derive(Debug, PartialEq, Copy, Clone, Eq, Serialize, Deserialize)]
pub enum Color {
    /// Couleur noire
    Black = 0x00,
    /// Couleur rouge
    Red = 0x01,
    /// Couleur verte
    Green = 0x02,
    /// Couleur jaune
    Yellow = 0x03,
    /// Couleur bleue
    Blue = 0x04,
    /// Couleur magenta
    Magenta = 0x05,
    /// Couleur cyan
    Cyan = 0x06,
    /// Couleur blanche
    White = 0x07,
}

impl Default for Color {
    fn default() -> Self {
        Color::Green
    }
}

impl ServoGroup {
    /// Désérialisation d'un JSON en `ServoGroup`
    pub fn from_json_slice(slice: &[u8]) -> Result<Self, ()> {
        let result = from_slice(slice);
        match result {
            Ok(t) => t,
            Err(_) => Err(()),
        }
    }
}

#[cfg(test)]
mod test {
    use super::{BlockingMode, Color, Control, Rotation, Servo};
    use heapless::consts::U256;
    use heapless::String;
    type N = U256;

    #[test]
    fn ser_deser_servo_speed() {
        let servo = Servo {
            id: 54,
            known_position: 67,
            control: Control::Speed,
            rotation: Rotation::CounterClockwise,
            data: 567,
            blocked: false,
            mode: BlockingMode::HoldOnBlock,
            color: Color::Blue,
        };
        let strd: String<N> = servo.to_string().unwrap();
        let _data =
            "{\"blocked\":false,\"color\":\"Blue\",\"control\":\"Speed\",\"rotation\":\"CounterClockwise\",\"data\":567,\"id\":54,\"known_position\":67,\"mode\":\"HoldOnBlock\"}"
        ;
        let servo2 = Servo::from_json_slice(strd.as_bytes()).unwrap();
        assert_eq!(servo, servo2);
    }

    #[test]
    fn ser_deser_servo_position() {
        let servo = Servo {
            id: 54,
            known_position: 67,
            control: Control::Position,
            //rotation: Rotation::CounterClockwise,
            data: 567,
            blocked: false,
            mode: BlockingMode::HoldOnBlock,
            color: Color::Blue,
            ..Default::default()
        };
        let _strd: String<N> = servo.to_string().unwrap();
        let data =
            "{\"blocked\":false,\"color\":\"Blue\",\"control\":\"Position\",\"rotation\":\"CounterClockwise\",\"data\":567,\"id\":54,\"known_position\":67,\"mode\":\"HoldOnBlock\"}"
        ;
        let servo2 = Servo::from_json_slice(data.as_bytes()).unwrap();
        assert_eq!(servo, servo2);
    }
}
