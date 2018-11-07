//! Représentation haut-niveau d'un servo-moteur.

use arrayvec::ArrayVec;
use transmission::ffi::{get_size_servo_frame, CSharedServos, ErrorParsing, FrameParsingTrait};
use transmission::Message;

/// Représentation d'un unique servo-moteur
#[derive(Debug, Default, Copy, Clone, Eq)]
pub struct Servo {
    /// Identifiant du servo-moteur.
    pub id: u8,
    /// Position actuelle du servo-moteur.
    pub known_position: u16,
    /// Commande du servo soit en angle soit en vitesse.
    pub control: Control,
    /// Retourne vrai si le servo-moteur est bloqué
    pub blocked: bool,
    /// Comportement du servo-moteur face à un blocage extérieur.
    pub mode: BlockingMode,
    /// Couleur émise par le servo-moteur.
    pub color: Color,
}

/// Un ensemble de au plus 8 servos-moteurs
#[derive(PartialEq, Debug, Clone)]
pub struct ServoGroup {
    /// Vecteur d'au plus 8 servos-moteurs
    pub servos: ArrayVec<[Servo; 8]>,
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
#[derive(Debug, PartialEq, Copy, Clone, Eq)]
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
#[derive(Debug, PartialEq, Copy, Clone, Eq)]
pub enum Control {
    /// Commande en vitesse.
    Speed(u16),
    /// Commande en position.
    Position(u16),
}

impl Default for Control {
    fn default() -> Self {
        Control::Position(512)
    }
}

/// Couleur émise par le servo-moteur.
#[derive(Debug, PartialEq, Copy, Clone, Eq)]
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
    /// Crée un nouveau groupe de servomoteur à partir d'un message.
    pub fn new(from_data: Message) -> Result<Self, ErrorParsing> {
        let read_servos: Result<CSharedServos, ErrorParsing> =
            FrameParsingTrait::read_frame(from_data);
        match read_servos {
            Ok(s) => Ok(s.into()),
            Err(e) => Err(e),
        }
    }

    /// Retourne la taille du message théorique, associé au nombre de servos présents.
    pub fn get_size_frame(nb_servos: u8) -> u8 {
        #[allow(unsafe_code)]
        unsafe {
            get_size_servo_frame(nb_servos)
        }
    }

    /// Renvoie un résultat contenant soit les octets correspondant à un message à renvoyer à la
    /// partie informatique, soit une erreur.
    pub fn into_bytes(self) -> Result<Message, ErrorParsing> {
        let ser: CSharedServos = self.into();
        ser.write_frame()
    }
}

impl Into<CSharedServos> for ServoGroup {
    fn into(self) -> CSharedServos {
        use transmission::ffi::*;
        let mut array = [CServo::default(); 8];
        let len = self.servos.len() as u8;

        for (i, servo) in self.servos.iter().enumerate() {
            let (cmd, cmd_type) = match servo.control {
                Control::Speed(val) => (val, 1),
                Control::Position(val) => (val, 0),
            };
            array[i] = CServo {
                id: servo.id,
                position: servo.known_position as cty::uint16_t,
                command: cmd as cty::uint16_t,
                command_type: cmd_type as cty::uint8_t,
                blocked: servo.blocked as cty::c_char,
                blocking_mode: servo.mode as cty::uint8_t,
                color: servo.color as cty::uint8_t,
            }
        }

        CSharedServos {
            servos: array,
            nb_servos: len,
            parsing_failed: 0,
        }
    }
}

impl Into<ServoGroup> for CSharedServos {
    fn into(self) -> ServoGroup {
        let mut array: ArrayVec<[Servo; 8]> = ArrayVec::<[Servo; 8]>::new();

        for servo in self.servos[0..self.nb_servos as usize].iter() {
            array.push(Servo {
                id: servo.id,
                /// Cette variable depuis l'informatique n'est pas intéressante
                known_position: servo.position,
                control: match servo.command_type {
                    0 => Control::Position(servo.command),
                    1 => Control::Speed(servo.command),
                    _ => unreachable!(),
                },
                blocked: servo.blocked != 0,
                mode: match servo.blocking_mode {
                    x if x == BlockingMode::Unblocking as u8 => BlockingMode::Unblocking,
                    x if x == BlockingMode::HoldOnblock as u8 => BlockingMode::HoldOnblock,
                    _ => unreachable!(),
                },
                color: match servo.color {
                    x if x == Color::Black as u8 => Color::Black,
                    x if x == Color::Red as u8 => Color::Red,
                    x if x == Color::Green as u8 => Color::Green,
                    x if x == Color::Yellow as u8 => Color::Yellow,
                    x if x == Color::Blue as u8 => Color::Blue,
                    x if x == Color::Magenta as u8 => Color::Magenta,
                    x if x == Color::Cyan as u8 => Color::Cyan,
                    x if x == Color::White as u8 => Color::White,
                    _ => unreachable!(), // réception de 3 bits seulement, soit 7 au maximum
                },
            });
        }
        ServoGroup { servos: array }
    }
}
