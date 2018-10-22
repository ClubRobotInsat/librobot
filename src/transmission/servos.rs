//! Représentation haut-niveau d'un servo-moteur
//! Un `Servo` peut être créé à partir de la représentation C d'un servo-moteur fournie sous forme d'octet.

use arrayvec::ArrayVec;
use transmission::ffi::{get_size_servo_frame, CSharedServos2019, ErrorParsing, FrameParsingTrait};
use transmission::Message;

/// Représentation d'un unique servo-moteur
#[derive(Debug, Copy, Clone, Eq)]
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

/// Commande du servo-moteur.
#[derive(Debug, PartialEq, Copy, Clone, Eq)]
pub enum Control {
    /// Commande en vitesse.
    Speed(u16),
    /// Commande en position.
    Position(u16),
}

/// Couleur émise par le servo-moteur.
#[derive(Debug, PartialEq, Copy, Clone, Eq)]
pub enum Color {
    /// Couleur noire
    BLACK = 0x00,
    /// Couleur rouge
    RED = 0x01,
    /// Couleur verte
    GREEN = 0x02,
    /// Couleur jaune
    YELLOW = 0x03,
    /// Couleur bleue
    BLUE = 0x04,
    /// Couleur magenta
    MAGENTA = 0x05,
    /// Couleur cyan
    CYAN = 0x06,
    /// Couleur blanche
    WHITE = 0x07,
}

impl ServoGroup {
    /// Crée un nouveau groupe de servomoteur à partir d'un message.
    pub fn new(from_data: Message) -> Result<Self, ErrorParsing> {
        let read_servos: Result<CSharedServos2019, ErrorParsing> =
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
}

impl Into<ServoGroup> for CSharedServos2019 {
    fn into(self) -> ServoGroup {
        let mut array: ArrayVec<[Servo; 8]> = ArrayVec::<[Servo; 8]>::new();

        for servo in self.servos[0..self.nb_servos as usize].iter() {
            array.push(Servo {
                id: servo.id,
                /// Cette variable depuis l'informatique n'est pas intéressante
                known_position: 0,
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
                    x if x == Color::BLACK as u8 => Color::BLACK,
                    x if x == Color::RED as u8 => Color::RED,
                    x if x == Color::GREEN as u8 => Color::GREEN,
                    x if x == Color::YELLOW as u8 => Color::YELLOW,
                    x if x == Color::BLUE as u8 => Color::BLUE,
                    x if x == Color::MAGENTA as u8 => Color::MAGENTA,
                    x if x == Color::CYAN as u8 => Color::CYAN,
                    x if x == Color::WHITE as u8 => Color::WHITE,
                    _ => unreachable!(), // réception de 3 bits seulement, soit 7 au maximum
                },
            });
        }
        ServoGroup { servos: array }
    }
}
