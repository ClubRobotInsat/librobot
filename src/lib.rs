#![no_std]
#![warn(missing_docs)]
#![deny(missing_debug_implementations)]
#![deny(unsafe_code)]
#![allow(dead_code)]

//! Librairie du club

// On inclue la librairie standard pour s'en servir dans les tests !
#[cfg(test)]
#[macro_use]
extern crate std;

pub extern crate arrayvec;
extern crate embedded_hal;
extern crate libc;
extern crate nb;

pub mod structs;
pub use structs::servos::Servos2019;

