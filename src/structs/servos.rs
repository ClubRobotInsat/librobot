//! Représentation haut-niveau d'un servo-moteur
//! Un `Servo2019` peut être créé à partir de la représentation C d'un servo-moteur

use structs::c_struct::{CSharedServos2019};
use structs::c_struct::{ErrorParsing, FrameParsingTrait, MsgVec};
use arrayvec::ArrayVec;

/// Représentation haut niveau d'un unique servo-moteur
#[derive(Debug, Copy, Clone)]
pub struct Servo2019 {
    /// Identifiant du servo-moteur. L'ID 0 est réservé pour spécifier l'abscence de servo-moteur.
    id: u8,
    /// Position actuelle du servo-moteur.
    position: u16,
    /// Commande du servo soit en angle soit en vitesse.
    control: Control,
    /// Retourne vrai si le servo-moteur est bloqué
    blocked: bool,
    /// Comportement du servo-moteur face à un blocage extérieur.
    mode: BlockingMode,
    /// Couleur émise par le servo-moteur.
    color: Color,
}

/// Ensemble de 8 servos-moteurs
#[derive(PartialEq, Debug, Clone)]
pub struct Servos2019 {
    /// Liste d'au plus 8 servos-moteurs
    pub list: ArrayVec<[Servo2019; 8]>,
}

/// Relation d'équivalence partielle pour le module `Servo2019`, utile pour le débug.
impl PartialEq for Servo2019 {
    fn eq(&self, other: &Servo2019) -> bool {
        self.id == other.id
            && (self.id == 0
            || (self.position == other.position
            && self.control == other.control
            && self.blocked == other.blocked
            && self.mode == other.mode
            && self.color == other.color))
    }
}

/// Relation d'équivalence pour le module `Servos2019` utile pour le débug (généré depuis PartialEq)
impl Eq for Servos2019 {}

/// Comportement du servo-moteur lorsqu'il est bloqué.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum BlockingMode {
    /// Le servo relâche la pression lorsqu'il est bloqué.
    Unblocking = 0,
    /// Le servo maintient un couple pour s'opposer au blocage.
    HoldOnblock = 1,
}

/// Commande du servo-moteur.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Control {
    /// Commande en vitesse.
    Speed(u16),
    /// Commande en position.
    Position(u16)
}

/// Couleur émise par le servo-moteur.
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum Color {
    /// couleur noire
    BLACK = 0x00,
    /// couleur rouge
    RED = 0x01,
    /// couleur verte
    GREEN = 0x02,
    /// couleur jaune
    YELLOW = 0x03,
    /// couleur bleue
    BLUE = 0x04,
    /// couleur magenta
    MAGENTA = 0x05,
    /// couleur cyan
    CYAN = 0x06,
    /// Couleur blanche
    WHITE = 0x07,
}

impl Servos2019 {
    fn new(from_data : MsgVec) -> Self {
        let read_servos : Result<CSharedServos2019, ErrorParsing> = FrameParsingTrait::read_frame(from_data);
        match read_servos {
            Ok(s) => s.into(),
            Err(..) => Servos2019 {
                list : ArrayVec::<[Servo2019; 8]>::new(),
            }
        }
    }
}

impl Into<Servos2019> for CSharedServos2019 {
    fn into(self) -> Servos2019 {
        let mut array : ArrayVec<[Servo2019; 8]> = ArrayVec::<[Servo2019; 8]>::new();

        for servo in self.servos.iter() {
            array.push(Servo2019 {
                id : servo.id,
                /// Cette variable depuis l'informatique n'est pas intéressante
                position : 0,
                control : match servo.command_type {
                    0 => Control::Position(servo.command),
                    1 => Control::Speed(servo.command),
                    _ => unreachable!()
                },
                blocked : servo.blocked != 0,
                mode : match servo.blocking_mode {
                    x if x == BlockingMode::Unblocking as u8 => BlockingMode::Unblocking,
                    x if x == BlockingMode::HoldOnblock as u8 => BlockingMode::HoldOnblock,
                    _ => unreachable!()
                },
                color : match servo.color {
                    x if x == Color::BLACK as u8 => Color::BLACK,
                    x if x == Color::RED as u8 => Color::RED,
                    x if x == Color::GREEN as u8 => Color::GREEN,
                    x if x == Color::YELLOW as u8 => Color::YELLOW,
                    x if x == Color::BLUE as u8 => Color::BLUE,
                    x if x == Color::MAGENTA as u8 => Color::MAGENTA,
                    x if x == Color::CYAN as u8 => Color::CYAN,
                    x if x == Color::WHITE as u8 => Color::WHITE,
                    _ => unreachable!() // réception de 3 bits seulement, soit 7 au maximum
                }
            });
        }
        Servos2019 {
            list: array,
        }
    }
}
