//! L'électronique et l'informatique communiquent grâce à des trames
//! et l'ensemble de l'état du robot est partagé dans chaque message
//!
//! Le robot possède des modules qui correspondent à un actionneur / capteur électronique
//! dont l'état correspondent à des sous-trames. Ils sont référencés par un ID unique (< 16).
//!
//! Les trames sont composées ainsi :
//! * un u16 pour connaître les modules présents (0x04 pour un module d'ID 3)
//! * un u16 pour chaque module et qui correspond à la taille occupée par la sous-trame associée
//! * la sous-trame de chaque sous-module
//!
//! Chaque sous-module est représenté par une sous-trame spécifique
//! dont le parsing se fait par un code en C
//! Ce choix permets de définir un unique couple de fonctions {lecture, écriture} pour chaque module
//! qui est utilisé à la fois en électronique et en informatique
//!
//! Ce fichier a pour vocation d'appeler du C dans le code Rust de manière sûre.
//!
//! Les structures représentatives de chaque module sont définies en C et en Rust.
//! Leur définition doivent être identiques à celles en C pour avoir accès aux fonctions de parsing.
//! Pour communiquer entre Rust et C, il faut utiliser la FFI de Rust
//! et utiliser la crate [cty](https://docs.rs/cty/).
//! De plus, il est nécessaire d'avoir la même représentation mémoire des structures
//! avec la représentation [repr(C)](https://doc.rust-lang.org/nomicon/other-reprs.html).

use core::marker::Sized;
use cty;

use arrayvec::ArrayVec;

use transmission::Message;

/// Représente la signature de la fonction C que l'on appelle pour transformer la frame en octets.
type WriteFunction<T> =
    unsafe extern "C" fn(*mut cty::uint8_t, cty::uint8_t, *const T) -> cty::uint8_t;

/// Représente la signature de la fonction C que l'on appelle pour transformer des octets en frame.
type ReadFunction<T> = unsafe extern "C" fn(*const cty::uint8_t, cty::uint8_t) -> T;

/// Permets de récupérer le type des structures pour un joli affichage dans les fonctions de parsing
pub trait TypeInfo {
    /// Retourne un string qui représente la nature de la structure
    fn type_of(&self) -> &'static str;
}

/// Représentation structurelle d'un unique servo-moteur
#[repr(C)]
#[derive(Debug, Default, Copy, Clone)]
pub struct CServo {
    /// Identifiant du servo-moteur. L'ID 0 est réservé pour spécifier l'abscence de servo-moteur.
    pub id: cty::uint8_t,
    /// Position actuelle du servo-moteur.
    pub position: cty::uint16_t,
    /// Ordre de position ou de vitesse donné par l'informatique.
    pub command: cty::uint16_t,
    /// Si égal à 0, la commande est en position ; si égal à 1 il s'agit d'un ordre de vitesse.
    pub command_type: cty::uint8_t,
    /// Si égal à 1, alors le servo-moteur est bloqué (il force).
    pub blocked: cty::c_char,
    /// HOLD_ON_BLOCKING = 1, UNBLOCKING = 0
    pub blocking_mode: cty::uint8_t,
    /// Couleur affichée sur le servo-moteur.
    pub color: cty::uint8_t,
}

/// Module complet de la gestion des servos-moteur
#[repr(C)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct CSharedServos {
    /// Ensemble des servos-moteurs.
    /// Il faut aussi modifier le code C pour avoir plus que 8 servos-moteur.
    pub servos: [CServo; 8],

    /// Le nombre de servos lus dans un message
    pub nb_servos: u8,

    /// Flag pour savoir si le parsing de la trame s'est bien réalisé par le C. 0 : OK, 1 : NOK.
    pub parsing_failed: cty::uint8_t,
}

/// Relation d'équivalence partielle pour le module `CServo`, utile pour le débug.
impl PartialEq for CServo {
    fn eq(&self, other: &CServo) -> bool {
        self.id == other.id
            && (self.id == 0
                || (self.position == other.position
                    && self.command == other.command
                    && self.command_type == other.command_type
                    && self.blocked == other.blocked
                    && self.blocking_mode == other.blocking_mode
                    && self.color == other.color))
    }
}

/// Relation d'équivalence pour le module `CServo` utile pour le débug (généré depuis PartialEq)
impl Eq for CSharedServos {}

/// Association d'un nom pour l'affichage dans le débug.
impl TypeInfo for CSharedServos {
    fn type_of(&self) -> &'static str {
        "CServos"
    }
}

/// Représentation structurelle d'un unique moteur asservi
/// TODO : l'informatique peut donner soit un ordre de rotation soit une consigne de nombre de tours
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct CControlledMotor {
    /// Identifiant du moteur asservi. L'ID 0 est réservé pour spécifier l'abscence de moteur.
    pub id: cty::uint8_t,
    /// Ordre angulaire donné par l'informatique.
    pub wanted_angle_position: cty::uint8_t,
    /// Ordre de nombre de tours donné par l'informatique.
    pub wanted_nb_turns: cty::uint8_t,
    /// Si le flag vaut 1, l'électronique spécifie que la commande est terminée.
    pub finished: cty::uint8_t,
    /// Si le flag vaut 1, l'informatique spécifie qu'un nouvel ordre a été donné
    /// L'électronique doit oublier les anciens ordres.
    pub new_command: cty::uint8_t,
}
/// Représentation structurelle d'un unique moteur non asservi
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct CUncontrolledMotor {
    /// Identifiant du moteur non asservi. L'ID 0 est réservé pour spécifier l'abscence de moteur.
    pub id: cty::uint8_t,
    /// Flag pour savoir si le moteur tourne ; 1 = ON, 0 = OFF.
    pub on_off: cty::uint8_t,
    /// SCHEDULE = 0, TRIGONOMETRIC = 1
    pub rotation: cty::uint8_t,
}
/// Représentation structurelle d'un unique brushless
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct CBrushless {
    /// Identifiant du brushless. L'ID 0 est réservé pour spécifier l'abscence de brushless.
    pub id: cty::uint8_t,
    /// Flag pour savoir si le brushless tourne ; 1 = ON, 0 = OFF.
    pub on_off: cty::uint8_t,
}

/// Module complet de la gestion des moteurs
#[repr(C)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct CSharedMotors {
    /// Ensemble des moteurs asservis.
    pub controlled_motors: [CControlledMotor; 8],
    /// Ensemble des moteurs non-asservis.
    pub uncontrolled_motors: [CUncontrolledMotor; 8],
    /// Ensemble des brushless.
    pub brushless: [CBrushless; 8],
    /// Flag pour savoir si le parsing de la trame s'est bien réalisé par le C. 0 : OK, 1 : NOK.
    pub parsing_failed: cty::uint8_t,
}

/// Relation d'équivalence partielle pour le module `CControlledMotor`, utile pour le débug.
impl PartialEq for CControlledMotor {
    fn eq(&self, other: &CControlledMotor) -> bool {
        self.id == other.id
            && (self.id == 0
                || (self.wanted_angle_position == other.wanted_angle_position
                    && self.wanted_nb_turns == other.wanted_nb_turns
                    && self.finished == other.finished
                    && self.new_command == other.new_command))
    }
}
/// Relation d'équivalence partielle pour le module `CUncontrolledMotor`, utile pour le débug.
impl PartialEq for CUncontrolledMotor {
    fn eq(&self, other: &CUncontrolledMotor) -> bool {
        self.id == other.id
            && (self.id == 0 || (self.on_off == other.on_off && self.rotation == other.rotation))
    }
}
/// Relation d'équivalence partielle pour le module `CBrushless`, utile pour le débug.
impl PartialEq for CBrushless {
    fn eq(&self, other: &CBrushless) -> bool {
        self.id == other.id && (self.id == 0 || (self.on_off == other.on_off))
    }
}
/// Relation d'équivalence pour le module `CMotor` utile pour le débug (généré depuis PartialEq)
impl Eq for CSharedMotors {}

/// Association d'un nom pour l'affichage dans le débug.
impl TypeInfo for CSharedMotors {
    fn type_of(&self) -> &'static str {
        "CMotors"
    }
}

/// Toutes les fonctions C doivent être définies ici pour le linkage
#[link(name = "SharedWithRust")]
extern "C" {
    /// Parsing du module des servos-moteur
    fn servo_read_frame(message: *const cty::uint8_t, size: cty::uint8_t) -> CSharedServos;
    fn servo_write_frame(
        buf: *mut cty::uint8_t,
        buf_size: cty::uint8_t,
        obj: *const CSharedServos,
    ) -> cty::uint8_t;
    pub(crate) fn get_size_servo_frame(nb_servos: cty::uint8_t) -> cty::uint8_t;

    /// Parsing du module des moteurs
    fn motor_read_frame(message: *const cty::uint8_t, size: cty::uint8_t) -> CSharedMotors;
    fn motor_write_frame(
        buf: *mut cty::uint8_t,
        buf_size: cty::uint8_t,
        obj: *const CSharedMotors,
    ) -> cty::uint8_t;
    pub(crate) fn get_size_motor_frame(
        nb_controlled: cty::uint8_t,
        nb_uncontrolled: cty::uint8_t,
        nb_brushless: cty::uint8_t,
    ) -> cty::uint8_t;

// TODO : récupérer les constantes partagées depuis le code C
/*pub static NBR_SERVOS: cty::uint8_t;
pub static NBR_CONTROLLED_MOTORS: cty::uint8_t;
pub static NBR_UNCONTROLLED_MOTORS: cty::uint8_t;
pub static NBR_BRUSHLESS: cty::uint8_t;*/
}

/// Fonctions de parsing génériques
/// Il faut `impl` chaque structure pour appeler ces fonctions lors du parsing
fn generic_read_frame<T>(
    message: Message,
    c_read_function: ReadFunction<T>,
) -> Result<T, ErrorParsing>
where
    T: FrameParsingTrait,
    T: TypeInfo,
{
    let mut buf = [0u8; 256];
    for (index, data) in message.iter().enumerate() {
        buf[index] = *data;
    }
    #[allow(unsafe_code)]
    let servo = unsafe { c_read_function((&buf).as_ptr(), message.len() as cty::uint8_t) };

    if servo.read_is_ok() {
        Ok(servo)
    } else {
        Err(ErrorParsing::BadPadding)
    }
}
fn generic_write_frame<T>(
    obj: &T,
    c_write_function: WriteFunction<T>,
) -> Result<Message, ErrorParsing>
where
    T: TypeInfo,
{
    let mut buf = [0u8; 256];

    #[allow(unsafe_code)]
    let size = unsafe { c_write_function((&mut buf).as_mut_ptr(), 255, obj) };

    if size == 0 {
        Err(ErrorParsing::BufferTooSmall)
    } else {
        let mut result = ArrayVec::<[u8; 256]>::new();

        for i in 0..size as usize {
            result.push(buf[i]);
        }

        Ok(result)
    }
}

/// Erreur levée lorsqu'un problème de parsing intervient en C
#[derive(Debug)]
pub enum ErrorParsing {
    /// La trame fournie en lecture est mal définie
    BadPadding,
    /// Le buffer fourni pour écrire une trame est trop petit
    BufferTooSmall,
}

/// Regroupements de méthodes permettant de sérialiser et déserialiser des Frames à partir d'un
/// flux d'octets.
pub trait FrameParsingTrait {
    /// Permet de transformer un buffer en message.
    fn read_frame(_msg: Message) -> Result<Self, ErrorParsing>
    where
        Self: Sized;
    /// Permet de transformer un message en octet.
    fn write_frame(&self) -> Result<Message, ErrorParsing>;
    /// Permet de vérifier la validité d'un message.
    fn read_is_ok(&self) -> bool;
}

impl FrameParsingTrait for CSharedServos {
    fn read_frame(msg: Message) -> Result<CSharedServos, ErrorParsing> {
        generic_read_frame(msg, servo_read_frame)
    }

    fn write_frame(&self) -> Result<Message, ErrorParsing> {
        generic_write_frame(self, servo_write_frame)
    }

    fn read_is_ok(&self) -> bool {
        self.parsing_failed == 0
    }
}

impl FrameParsingTrait for CSharedMotors {
    fn read_frame(msg: Message) -> Result<CSharedMotors, ErrorParsing> {
        generic_read_frame(msg, motor_read_frame)
    }

    fn write_frame(&self) -> Result<Message, ErrorParsing> {
        generic_write_frame(self, motor_write_frame)
    }

    fn read_is_ok(&self) -> bool {
        self.parsing_failed == 0
    }
}

#[cfg(test)]
mod tests {

    use arrayvec::ArrayVec;
    use transmission::ffi::*;
    use transmission::servo::*;

    #[test]
    fn buffer_overflow_servo_frame() {
        let data = [
            0x6f, 0x6f, 0xb3, 0xb3, 0x0, 0x6, 0xbf, 0x1c, 0xfb, 0xe, 0xd7, 0x2, 0x8a, 0x3f, 0x0,
            0xd, 0xff, 0xfb, 0x2, 0xd, 0x0, 0x3, 0xff, 0xdd, 0x86, 0x86, 0x3, 0x3, 0xfd, 0x1d, 0x0,
            0x3, 0xfc, 0xff, 0x18, 0x1d, 0x23, 0x2, 0x7f, 0x21, 0x3, 0x0, 0xff, 0xa, 0xd, 0xe7,
            0x3, 0xff, 0x1d, 0xda, 0xff, 0xff, 0xff, 0x29, 0x0, 0x2c, 0x2c, 0xff, 0x7e, 0x0, 0xff,
            0x2c, 0x2c, 0x2c, 0x2c, 0x15, 0x2, 0x3d, 0xd, 0xff, 0x7e, 0x0, 0xff, 0xff, 0x3f, 0xff,
            0x2, 0xd, 0x0, 0x3, 0xff, 0xd, 0x0, 0x3, 0xa7, 0x3, 0xff, 0x1d, 0x30, 0xff, 0x1d, 0x86,
            0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86,
            0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86,
            0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0xff, 0xff, 0xff, 0x1d,
            0x2, 0x12, 0x86, 0x86, 0x86, 0x86, 0x86, 0x86, 0xff, 0xff, 0xff, 0x1d, 0x2, 0x12, 0xa7,
            0xff, 0x86, 0x0, 0x86, 0x86, 0x0,
        ];
        let mut bytes = Message::new();
        for b in data.into_iter() {
            bytes.push(*b);
        }
        let _test = CSharedServos::read_frame(bytes);
    }

    #[test]
    fn buffer_overflow_servo_frame_2() {
        let data = [
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3, 0x2, 0x1, 0x1, 0x0, 0x0, 0x8, 0x88, 0x0, 0xba,
            0x2, 0x1, 0x8, 0xf7, 0x19, 0x8, 0xff, 0xe9, 0x20, 0xae, 0x0, 0x0, 0x80, 0xff, 0x2, 0x8,
            0xff, 0xff, 0x50, 0x50, 0x50, 0x50, 0xe7, 0x50, 0x19, 0x0, 0x38, 0x20, 0x50, 0x20,
            0xff, 0x19, 0x21, 0x3, 0x2, 0xfa, 0x1, 0xff, 0x1, 0x2, 0x8, 0x8, 0x88, 0x0, 0x0, 0x2,
            0xae, 0x2, 0xae, 0x2, 0x0, 0x0, 0x8, 0x8, 0x88, 0x2, 0x0, 0xba, 0x5b, 0xff, 0x2, 0x2,
            0x0, 0x2, 0xae, 0x3, 0x2, 0x3, 0x2, 0x1, 0x19, 0x8, 0x2, 0x0, 0x0, 0xdd, 0x8, 0x2, 0x0,
            0x0, 0x8, 0xd, 0x20, 0x38, 0x20, 0x88, 0x1, 0x19, 0x8, 0x0, 0x0, 0x0, 0xff, 0xff, 0x19,
            0x0, 0x37, 0x20, 0xff, 0xfe, 0x0, 0x0, 0x19, 0x8, 0x23, 0xff, 0x0, 0x25, 0x80, 0xff,
            0xa4, 0x0, 0x2, 0x0, 0x0, 0x0, 0x1d, 0xfe, 0xff, 0xff, 0x1c, 0x2, 0xff, 0x23, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3c, 0x3c, 0x2, 0x1c, 0x2, 0xff, 0x23,
            0xff, 0xa4, 0x8, 0x1, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xa7, 0xd0, 0xd0, 0x8,
            0xd, 0x20, 0x38, 0x80, 0xff, 0xa4, 0x20, 0x0, 0x3, 0x2, 0x1, 0x1, 0x0, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf7, 0xd, 0x8, 0xff, 0x17, 0xd7, 0xfe, 0x1, 0x19,
            0x8, 0x2, 0xd0, 0xd0, 0xd0, 0xd0, 0x3b, 0xd1, 0xd0, 0xd0, 0xd0, 0xd0, 0xd0, 0xff, 0xe6,
            0xff, 0x23, 0xfc, 0x2, 0x8, 0x8, 0x88, 0x2, 0x0, 0xba, 0x5b, 0xff, 0x8, 0x88, 0x0,
            0x2a, 0x19, 0x8, 0xff, 0x19, 0x8, 0x19, 0x1, 0x19, 0xff, 0xff, 0xff, 0x2, 0x2, 0xff,
            0x1c, 0x0, 0x8, 0x88,
        ];
        let mut bytes = Message::new();
        for b in data.into_iter() {
            bytes.push(*b);
        }
        let _test = CSharedServos::read_frame(bytes);
    }

    #[test]
    fn servos() {
        let servo_empty = CServo {
            id: 0,
            position: 0,
            command: 0,
            command_type: 0,
            blocked: 0,
            blocking_mode: 0,
            color: 0,
        };
        assert_eq!(
            servo_empty,
            CServo {
                id: 0,
                position: 1,
                command: 2,
                command_type: 1,
                blocked: 4,
                blocking_mode: 5,
                color: 6,
            }
        );
        let servo1 = CServo {
            id: 1,
            position: 512,
            command: 162,
            command_type: 1,
            blocked: 0,
            blocking_mode: 0,
            color: 5,
        };
        assert_eq!(servo1, servo1);
        let servo3 = CServo {
            id: 3,
            position: 1000,
            command: 10,
            command_type: 1,
            blocked: 1,
            blocking_mode: 1,
            color: 3,
        };
        assert_ne!(servo1, servo3);

        let mut array = [servo_empty; 8];
        array[0] = servo1;
        array[1] = servo3;

        let struct_before = CSharedServos {
            servos: array,
            parsing_failed: 0,
            nb_servos: 2,
        };

        let written_frame = struct_before.write_frame();
        assert!(written_frame.is_ok());
        let read_frame =
            CSharedServos::read_frame(written_frame.unwrap_or(ArrayVec::<[u8; 256]>::new()));
        assert!(read_frame.is_ok());
        let struct_after = read_frame.unwrap_or(struct_before);
        // Les éléments ne sont pas dans le même ordre mais les structures sont équivalentes
        for servo in &struct_before.servos {
            assert!(&struct_after
                .servos
                .iter()
                .find(|elem| *elem == servo)
                .is_some());
        }

        assert_eq!(ServoGroup::get_size_frame(5), 31);
    }

    #[test]
    fn servo_reversibility() {
        let mut tab = [Servo::default(); 8];
        tab[0] = Servo {
            id: 1,
            known_position: 0,
            control: Control::Position(0),
            blocked: false,
            mode: BlockingMode::HoldOnblock,
            color: Color::Blue,
        };
        tab[1] = Servo {
            id: 89,
            known_position: 25,
            control: Control::Speed(56),
            blocked: false,
            mode: BlockingMode::Unblocking,
            color: Color::Black,
        };
        tab[2] = Servo {
            id: 12,
            known_position: 1023,
            control: Control::Speed(80),
            blocked: true,
            mode: BlockingMode::Unblocking,
            color: Color::Red,
        };
        tab[3] = Servo {
            id: 255,
            known_position: 512,
            control: Control::Position(12),
            blocked: false,
            mode: BlockingMode::HoldOnblock,
            color: Color::Green,
        };
        tab[4] = Servo {
            id: 254,
            known_position: 1,
            control: Control::Speed(1023),
            blocked: false,
            mode: BlockingMode::Unblocking,
            color: Color::Red,
        };
        tab[5] = Servo {
            id: 127,
            known_position: 999,
            control: Control::Position(1023),
            blocked: true,
            mode: BlockingMode::HoldOnblock,
            color: Color::Magenta,
        };
        tab[6] = Servo {
            id: 45,
            known_position: 0,
            control: Control::Position(512),
            blocked: false,
            mode: BlockingMode::Unblocking,
            color: Color::Black,
        };
        tab[7] = Servo {
            id: 128,
            known_position: 512,
            control: Control::Speed(0),
            blocked: true,
            mode: BlockingMode::HoldOnblock,
            color: Color::White,
        };
        let mut vec: ArrayVec<[Servo; 8]> = ArrayVec::new();
        for elem in &tab[1..=5] {
            vec.push(*elem);
        }
        let servos = ServoGroup { servos: vec };
        let original = servos.clone();
        let bytes = servos.into_bytes().unwrap();
        let new = ServoGroup::new(bytes).unwrap();
        assert_eq!(new, original);
    }

    #[test]
    fn motors() {
        let controlled_empty = CControlledMotor {
            id: 0,
            wanted_angle_position: 0,
            wanted_nb_turns: 0,
            finished: 0,
            new_command: 0,
        };
        assert_eq!(
            controlled_empty,
            CControlledMotor {
                id: 0,
                wanted_angle_position: 1,
                wanted_nb_turns: 2,
                finished: 3,
                new_command: 4,
            }
        );
        let controlled_1 = CControlledMotor {
            id: 1,
            wanted_angle_position: 213,
            wanted_nb_turns: 2,
            finished: 0,
            new_command: 1,
        };
        assert_eq!(controlled_1, controlled_1);
        let controlled_3 = CControlledMotor {
            id: 3,
            wanted_angle_position: 12,
            wanted_nb_turns: 5,
            finished: 1,
            new_command: 0,
        };
        assert_ne!(controlled_1, controlled_3);

        let uncontrolled_empty = CUncontrolledMotor {
            id: 0,
            on_off: 0,
            rotation: 0,
        };
        assert_eq!(
            uncontrolled_empty,
            CUncontrolledMotor {
                id: 0,
                on_off: 1,
                rotation: 0,
            }
        );
        let uncontrolled_1 = CUncontrolledMotor {
            id: 1,
            on_off: 0,
            rotation: 0,
        };
        assert_eq!(uncontrolled_1, uncontrolled_1);

        let brushless_empty = CBrushless { id: 0, on_off: 0 };
        assert_eq!(brushless_empty, CBrushless { id: 0, on_off: 1 });
        let brushless_1 = CBrushless { id: 1, on_off: 0 };
        assert_eq!(brushless_1, brushless_1);
        let brushless_5 = CBrushless { id: 5, on_off: 0 };
        let brushless_6 = CBrushless { id: 6, on_off: 1 };

        let mut array_controlled = [controlled_empty; 8];
        array_controlled[1] = controlled_1;
        array_controlled[3] = controlled_3;

        let mut array_uncontrolled = [uncontrolled_empty; 8];
        array_uncontrolled[1] = uncontrolled_1;

        let mut array_brushless = [brushless_empty; 8];
        array_brushless[1] = brushless_1;
        array_brushless[5] = brushless_5;
        array_brushless[6] = brushless_6;

        let struct_before = CSharedMotors {
            controlled_motors: array_controlled,
            uncontrolled_motors: array_uncontrolled,
            brushless: array_brushless,

            parsing_failed: 0,
        };

        let written_frame = struct_before.write_frame();
        assert!(written_frame.is_ok());
        let read_frame =
            CSharedMotors::read_frame(written_frame.unwrap_or(ArrayVec::<[u8; 256]>::new()));
        assert!(read_frame.is_ok());
        let struct_after = read_frame.unwrap_or(struct_before);

        for motor in &struct_before.controlled_motors {
            assert!(&struct_after
                .controlled_motors
                .iter()
                .find(|elem| *elem == motor)
                .is_some());
        }
        for motor in &struct_before.uncontrolled_motors {
            assert!(&struct_after
                .uncontrolled_motors
                .iter()
                .find(|elem| *elem == motor)
                .is_some());
        }
        for motor in &struct_before.brushless {
            assert!(&struct_after
                .brushless
                .iter()
                .find(|elem| *elem == motor)
                .is_some());
        }
    }

}
