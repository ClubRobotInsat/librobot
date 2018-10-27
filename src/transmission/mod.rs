//! Ensemble des types, fonctions et traits qui permettent la communication electronique<-> informatique.
//!
//! Le code qui permet de transformer des octets en structures et inversemment est écrit en C.
//! Pour chaque type de messages, il y a une structure C (que vous ne voyez pas car elle est cachée)
//! et une structure Rust qui correspond exactement à celle en C.
//!
//! Au niveau de la librairie vous ne voyez ni l'une, ni l'autre car on expose une structure Rust plus
//! idiomatique.
//!
//! # Exemple
//!
//! Voici un exemple de communication PC-Elec :
//! ```c++
//! +----------------+           +--------------------+
//! |                |           |                    |
//! |  Raspberry PI  |           |  Microcontrolleur  |
//! |                |           |                    |
//! +--------+-------+           +----------+---------+
//!          |                              |
//!          |  AC DC AB BA 01 05 06 00     |
//!          | +--------------------------> |
//!          |                              |
//!          |  AC DC AB BB 01 00 00        |
//!          | <--------------------------+ |
//!          |                              |
//!          |  AC DC AB BA 05 46 02 11 77  |
//!          | <--------------------------+ |
//!          |                              |
//!          v(t)                           v(t)
//! ```

pub use transmission::servos::*;

use arrayvec::ArrayVec;

#[macro_use]
pub mod frame;
mod ffi;
pub mod frame_reader;
pub mod servos;

#[cfg(test)]
mod tests;

/// Taille maximale du message véhiculé par la trame
pub const FRAME_MAX_SIZE: usize = frame_reader::FRAME_READER_INTERNAL_BUFFER_SIZE /* - 6*/;
/// Un message est un tableau de 256 octets.
pub type Message = ArrayVec<[u8; FRAME_MAX_SIZE]>;
