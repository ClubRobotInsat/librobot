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
extern crate uom as unit;

extern crate embedded_hal;
extern crate libc;
extern crate nb;

pub mod transmission;
pub use transmission::*;

pub mod navigation;
