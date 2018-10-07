extern crate libc;
use core::marker::Sized;
use libc::uint8_t;

use arrayvec::ArrayVec;

type MsgVec = ArrayVec<[u8;1024]>;

/// Représente la signature de la fonction C que l'on appelle pour transformer la frame en octets.
type WriteFunction<T> = unsafe extern "C" fn(*mut uint8_t, uint8_t, *const T) -> uint8_t;

/// Représente la signature de la fonction C que l'on appelle pour transformer des octets en frame.
type ReadFunction<T> = unsafe extern "C" fn(*const uint8_t, uint8_t) -> T;

/// Ca permets de récupérer le type des structures pour un joli affichage dans les fonctions de parsing
pub trait TypeInfo {
    fn type_of(&self) -> &'static str;
}

/// La définition de cette structure doit être identique à celle de C
/// L'intérêt de cette même définition permet d'avoir accès aux fonctions de parsing
/// définies uniquement en C
/// Afin de pouvoir communiquer entre Rust et C, il faut utiliser le crate
/// [libc](https://docs.rs/libc/0.2.43/libc/index.html) et utiliser la même représentation mémoire
/// qu'en C avec [repr(C)](https://doc.rust-lang.org/nomicon/other-reprs.html)
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Servo2019 {
    pub id: libc::uint8_t,
    pub position: libc::uint16_t,
    pub wanted_position: libc::uint16_t,
    pub speed: libc::uint8_t,
    pub blocked: libc::c_char,
    pub blocking_mode: libc::uint8_t,
    pub color: libc::uint8_t,
}

#[repr(C)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct SharedServos2019 {
    pub servos: [Servo2019; 8],
    pub parsing_failed: libc::uint8_t,
}

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

impl Eq for SharedServos2019 {}

impl TypeInfo for SharedServos2019 {
    fn type_of(&self) -> &'static str {
        "Servos2019"
    }
}

#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct ControlledMotor2019 {
    pub id: libc::uint8_t,
    pub wanted_angle_position: libc::uint8_t,
    pub wanted_nb_turns: libc::uint8_t,
    pub finished: libc::uint8_t,
    pub new_command: libc::uint8_t,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct UncontrolledMotor2019 {
    pub id: libc::uint8_t,
    pub on_off: libc::uint8_t,
    pub rotation: libc::uint8_t,
}
#[repr(C)]
#[derive(Debug, Copy, Clone)]
pub struct Brushless2019 {
    pub id: libc::uint8_t,
    pub on_off: libc::uint8_t,
}
#[repr(C)]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct SharedMotors2019 {
    pub controlled_motors: [ControlledMotor2019; 8],
    pub uncontrolled_motors: [UncontrolledMotor2019; 8],
    pub brushless: [Brushless2019; 8],
    pub parsing_failed: libc::uint8_t,
}

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
impl PartialEq for UncontrolledMotor2019 {
    fn eq(&self, other: &UncontrolledMotor2019) -> bool {
        self.id == other.id
            && (self.id == 0 || (self.on_off == other.on_off && self.rotation == other.rotation))
    }
}
impl PartialEq for Brushless2019 {
    fn eq(&self, other: &Brushless2019) -> bool {
        self.id == other.id && (self.id == 0 || (self.on_off == other.on_off))
    }
}
impl Eq for SharedMotors2019 {}

impl TypeInfo for SharedMotors2019 {
    fn type_of(&self) -> &'static str {
        "Motors2019"
    }
}

#[link(name="SharedWithRust")]
extern "C" {
    pub fn servo_read_frame(message: *const libc::uint8_t, size: libc::uint8_t)
        -> SharedServos2019;
    pub fn servo_write_frame(
        buf: *mut libc::uint8_t,
        buf_size: libc::uint8_t,
        obj: *const SharedServos2019,
    ) -> libc::uint8_t;

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
