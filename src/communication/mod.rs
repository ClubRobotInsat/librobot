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


mod ffi;
pub mod servos;
pub use communication::servos::ServoGroup;
pub use communication::servos::Servo;

#[cfg(test)]
mod tests;
