#![no_std]
#![warn(missing_docs)]
#![deny(missing_debug_implementations)]
#![deny(unsafe_code)]
#![allow(dead_code)]

//! La librairie du club pour les µ-controlleurs arm.

#[cfg(test)]
#[macro_use]
extern crate std;

pub extern crate arrayvec;
extern crate embedded_hal;
extern crate libc;
extern crate nb;

pub mod communication;
pub use communication::*;

#[macro_use]
pub mod frame;
pub mod frame_reader;
