//! Représentation haut-niveau d'un servo-moteur.

use arrayvec::ArrayVec;
use heapless::{ArrayLength, String};
use serde_json_core::de::{from_slice, Error as DError};
use serde_json_core::ser::{to_string, Error as SError};

use transmission::ffi::{get_size_servo_frame, CSharedServos, ErrorParsing, FrameParsingTrait};
use transmission::Message;

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
    pub data: u16,
    /// Retourne vrai si le servo-moteur est bloqué
    pub blocked: bool,
    /// Comportement du servo-moteur face à un blocage extérieur.
    pub mode: BlockingMode,
    /// Couleur émise par le servo-moteur.
    pub color: Color,
}

impl Servo {
    pub fn from_json_slice(slice: &[u8]) -> Result<Self, DError> {
        from_slice(slice)
    }

    pub fn to_string<B>(&self) -> Result<String<B>, SError>
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
    HoldOnblock = 1,
}

impl Default for BlockingMode {
    fn default() -> Self {
        BlockingMode::Unblocking
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
    pub fn from_json_slice(slice: &[u8]) -> Result<Self, ()> {
        let result = from_slice(slice);
        match result {
            Ok(t) => t,
            Err(_) => Err(()),
        }
    }
    /*
    /// Crée un nouveau groupe de servomoteur à partir d'un message.
    pub fn from_message(from_data: Message) -> Result<Self, ErrorParsing> {
        let read_servos: Result<CSharedServos, ErrorParsing> =
            FrameParsingTrait::read_frame(from_data);
        match read_servos {
            Ok(s) => Ok(s.into()),
            Err(e) => Err(e),
        }
    }*/

    /// Retourne la taille du message théorique, associé au nombre de servos présents.
    pub fn get_size_frame(nb_servos: u8) -> u8 {
        #[allow(unsafe_code)]
        unsafe {
            get_size_servo_frame(nb_servos)
        }
    }
    /*
    /// Renvoie un résultat contenant soit les octets correspondant à un message à renvoyer à la
    /// partie informatique, soit une erreur.
    pub fn into_bytes(self) -> Result<Message, ErrorParsing> {
        let ser: CSharedServos = self.into();
        ser.write_frame()
    }
    */
}

#[cfg(test)]
mod test {
    use super::{BlockingMode, Color, Control, Servo};
    use heapless::consts::U256;
    use heapless::String;
    type N = U256;

    #[test]
    fn ser_deser_servo() {
        let servo = Servo {
            id: 54,
            known_position: 67,
            control: Control::Speed,
            data: 567,
            blocked: false,
            mode: BlockingMode::HoldOnblock,
            color: Color::Blue,
        };
        let strd: String<N> = servo.to_string().unwrap();
        let data =
            "{\"blocked\":false,\"color\":\"Blue\",\"control\":\"Speed\",\"data\":567,\"id\":54,\"known_position\":67,\"mode\":\"HoldOnblock\"}"
        ;
        let servo2 = Servo::from_json_slice(data.as_bytes()).unwrap();
        assert_eq!(servo, servo2);
    }
}
