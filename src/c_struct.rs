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
//! et utiliser le crate [libc](https://docs.rs/libc/0.2.43/libc/index.html).
//! De plus, il est nécessaire d'avoir la même représentation mémoire des structures
//! avec la représentation [repr(C)](https://doc.rust-lang.org/nomicon/other-reprs.html).

extern crate libc;
use core::marker::Sized;
use libc::uint8_t;

use arrayvec::ArrayVec;

type MsgVec = ArrayVec<[u8;1024]>;

/// Représente la signature de la fonction C que l'on appelle pour transformer la frame en octets.
type WriteFunction<T> = unsafe extern "C" fn(*mut uint8_t, uint8_t, *const T) -> uint8_t;

/// Représente la signature de la fonction C que l'on appelle pour transformer des octets en frame.
type ReadFunction<T> = unsafe extern "C" fn(*const uint8_t, uint8_t) -> T;

/// Permets de récupérer le type des structures pour un joli affichage dans les fonctions de parsing
pub trait TypeInfo {
    fn type_of(&self) -> &'static str;
}

/// Représentation structurelle d'un unique servo-moteur
/// TODO : l'informatique peut donner soit un ordre de position soit de vitesse (union)
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Servo2019 {
    /// Identifiant du servo-moteur. L'ID 0 est réservé pour spécifier l'abscence de servo-moteur.
    pub id: libc::uint8_t,
    /// Position actuelle du servo-moteur.
    pub position: libc::uint16_t,
    /// Ordre de position donné par l'informatique.
    pub wanted_position: libc::uint16_t,
    /// Ordre de vitesse donné par l'informatique.
    pub speed: libc::uint8_t,
    /// Si égal à 1, alors le servo-moteur est bloqué (il force).
    pub blocked: libc::c_char,
    /// HOLD_ON_BLOCKING = 1, UNBLOCKING = 0
    pub blocking_mode: libc::uint8_t,
    /// Couleur affichée sur le servo-moteur.
    pub color: libc::uint8_t,
}

/// Module complet de la gestion des servos-moteur
#[repr(C)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct SharedServos2019 {
    /// Ensemble des servos-moteurs.
    /// Il faut aussi modifier le code C pour avoir plus que 8 servos-moteur.
    pub servos: [Servo2019; 8],
    /// Flag pour savoir si le parsing de la trame s'est bien réalisé par le C. 0 : OK, 1 : NOK.
    pub parsing_failed: libc::uint8_t,
}

/// Relation d'équivalence partielle pour le module `Servo2019`, utile pour le débug.
impl PartialEq for Servo2019 {
    fn eq(&self, other: &Servo2019) -> bool {
        self.id == other.id
            && (self.id == 0
                || (self.position == other.position && self.wanted_position == other.wanted_position
                    && self.speed == other.speed
                    && self.blocked == other.blocked
                    && self.blocking_mode == other.blocking_mode
                    && self.color == other.color))
    }
}

/// Relation d'équivalence pour le module `Servo2019`, utile pour le débug (généré depuis PartialEq)
impl Eq for SharedServos2019 {}

/// Association d'un nom pour l'affichage dans le débug.
impl TypeInfo for SharedServos2019 {
    fn type_of(&self) -> &'static str {
        "Servos2019"
    }
}

/// Représentation structurelle d'un unique moteur asservi
/// TODO : l'informatique peut donner soit un ordre de rotation soit une consigne de nombre de tours
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct ControlledMotor2019 {
    /// Identifiant du moteur asservi. L'ID 0 est réservé pour spécifier l'abscence de moteur.
    pub id: libc::uint8_t,
    /// Ordre angulaire donné par l'informatique.
    pub wanted_angle_position: libc::uint8_t,
    /// Ordre de nombre de tours donné par l'informatique.
    pub wanted_nb_turns: libc::uint8_t,
    /// Si le flag vaut 1, l'électronique spécifie que la commande est terminée.
    pub finished: libc::uint8_t,
    /// Si le flag vaut 1, l'informatique spécifie qu'un nouvel ordre a été donné
    /// L'électronique doit oublier les anciens ordres.
    pub new_command: libc::uint8_t,
}
/// Représentation structurelle d'un unique moteur non asservi
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct UncontrolledMotor2019 {
    /// Identifiant du moteur non asservi. L'ID 0 est réservé pour spécifier l'abscence de moteur.
    pub id: libc::uint8_t,
    /// Flag pour savoir si le moteur tourne ; 1 = ON, 0 = OFF.
    pub on_off: libc::uint8_t,
    /// SCHEDULE = 0, TRIGONOMETRIC = 1
    pub rotation: libc::uint8_t,
}
/// Représentation structurelle d'un unique brushless
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Brushless2019 {
    /// Identifiant du brushless. L'ID 0 est réservé pour spécifier l'abscence de brushless.
    pub id: libc::uint8_t,
    /// Flag pour savoir si le brushless tourne ; 1 = ON, 0 = OFF.
    pub on_off: libc::uint8_t,
}

/// Module complet de la gestion des moteurs
#[repr(C)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct SharedMotors2019 {
    /// Ensemble des moteurs asservis.
    pub controlled_motors: [ControlledMotor2019; 8],
    /// Ensemble des moteurs non-asservis.
    pub uncontrolled_motors: [UncontrolledMotor2019; 8],
    /// Ensemble des brushless.
    pub brushless: [Brushless2019; 8],
    /// Flag pour savoir si le parsing de la trame s'est bien réalisé par le C. 0 : OK, 1 : NOK.
    pub parsing_failed: libc::uint8_t,
}

/// Relation d'équivalence partielle pour le module `ControlledMotor2019`, utile pour le débug.
impl PartialEq for ControlledMotor2019 {
    fn eq(&self, other: &ControlledMotor2019) -> bool {
        self.id == other.id
            && (self.id == 0
                || (self.wanted_angle_position == other.wanted_angle_position
                    && self.wanted_nb_turns == other.wanted_nb_turns
                    && self.finished == other.finished
                    && self.new_command == other.new_command))
    }
}
/// Relation d'équivalence partielle pour le module `UncontrolledMotor2019`, utile pour le débug.
impl PartialEq for UncontrolledMotor2019 {
    fn eq(&self, other: &UncontrolledMotor2019) -> bool {
        self.id == other.id
            && (self.id == 0 || (self.on_off == other.on_off && self.rotation == other.rotation))
    }
}
/// Relation d'équivalence partielle pour le module `Brushless2019`, utile pour le débug.
impl PartialEq for Brushless2019 {
    fn eq(&self, other: &Brushless2019) -> bool {
        self.id == other.id && (self.id == 0 || (self.on_off == other.on_off))
    }
}
/// Relation d'équivalence pour le module `Motor2019`, utile pour le débug (généré depuis PartialEq)
impl Eq for SharedMotors2019 {}

/// Association d'un nom pour l'affichage dans le débug.
impl TypeInfo for SharedMotors2019 {
    fn type_of(&self) -> &'static str {
        "Motors2019"
    }
}

/// Toutes les fonctions C doivent être définies ici pour le linkage
#[link(name="SharedWithRust")]
extern "C" {
    /// Parsing du module des servos-moteur
    pub fn servo_read_frame(message: *const libc::uint8_t, size: libc::uint8_t)
        -> SharedServos2019;
    pub fn servo_write_frame(
        buf: *mut libc::uint8_t,
        buf_size: libc::uint8_t,
        obj: *const SharedServos2019,
    ) -> libc::uint8_t;

    /// Parsing du module des moteurs
    pub fn motor_read_frame(message: *const libc::uint8_t, size: libc::uint8_t)
        -> SharedMotors2019;
    pub fn motor_write_frame(
        buf: *mut libc::uint8_t,
        buf_size: libc::uint8_t,
        obj: *const SharedMotors2019,
    ) -> libc::uint8_t;

// TODO : récupérer les constantes partagées depuis le code C
    /*pub static NBR_SERVOS: libc::uint8_t;
    pub static NBR_CONTROLLED_MOTORS: libc::uint8_t;
    pub static NBR_UNCONTROLLED_MOTORS: libc::uint8_t;
    pub static NBR_BRUSHLESS: libc::uint8_t;*/
}

/// Fonctions de parsing génériques
/// Il faut `impl` chaque structure pour appeler ces fonctions lors du parsing
pub fn generic_read_frame<T>(
    message: MsgVec,
    c_read_function: ReadFunction<T>,
) -> Result<T, ErrorParsing>
where
    T: FrameParsingTrait,
    T: TypeInfo,
{
    #[allow(unsafe_code)]
    let servo = unsafe { c_read_function(message.as_ptr(), message.len() as uint8_t) };

    if servo.read_is_ok() {
        Ok(servo)
    } else {
        Err(ErrorParsing::BadPadding)
    }
}
fn generic_write_frame<T>(
    obj: &T,
    c_write_function: WriteFunction<T>,
) -> Result<MsgVec, ErrorParsing>
where
    T: TypeInfo,
{
    let mut buf = MsgVec::new();

    #[allow(unsafe_code)]
    let size = unsafe { c_write_function(buf.as_mut_ptr(), buf.capacity() as uint8_t, obj) };

    if size == 0 {
        Err(ErrorParsing::BufferTooSmall)
    } else {
        /*unsafe {
            buf.set_len(size as usize);
        }*/
        Ok(buf)
    }
}

// TODO : Documentation
#[derive(Debug)]
pub enum ErrorParsing {
    BadPadding,
    BufferTooSmall,
}

/// Regroupements de méthodes permettant de sérialiser et déserialiser des Frames à partir d'un
/// flux d'octets.
pub trait FrameParsingTrait {
    /// Permet de transformer un buffer en message.
    fn read_frame(_msg: MsgVec) -> Result<Self, ErrorParsing>
    where
        Self: Sized;
    /// Permet de transformer un message en octet.
    fn write_frame(&self) -> Result<MsgVec, ErrorParsing>;
    /// Permet de vérifier la validité d'un message.
    fn read_is_ok(&self) -> bool;
}

impl FrameParsingTrait for SharedServos2019 {
    fn read_frame(msg: MsgVec) -> Result<SharedServos2019, ErrorParsing> {
        generic_read_frame(msg, servo_read_frame)
    }

    fn write_frame(&self) -> Result<MsgVec, ErrorParsing> {
        generic_write_frame(self, servo_write_frame)
    }

    fn read_is_ok(&self) -> bool {
        self.parsing_failed == 0
    }
}

impl FrameParsingTrait for SharedMotors2019 {
    fn read_frame(msg: MsgVec) -> Result<SharedMotors2019, ErrorParsing> {
        generic_read_frame(msg, motor_read_frame)
    }

    fn write_frame(&self) -> Result<MsgVec, ErrorParsing> {
        generic_write_frame(self, motor_write_frame)
    }

    fn read_is_ok(&self) -> bool {
        self.parsing_failed == 0
    }
}
