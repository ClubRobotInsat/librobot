#![no_std]
#![warn(missing_docs)]
#![deny(missing_debug_implementations)]
#![warn(unsafe_code)]
#![allow(dead_code)]

//! La librairie du club pour les µ-controlleurs arm.

#[cfg(test)]
#[macro_use]
extern crate std;

pub extern crate arrayvec;

extern crate cty;
extern crate embedded_hal;
extern crate libm;
extern crate nb;
extern crate qei;

pub use transmission::*;

pub mod navigation;
pub mod transmission;
pub mod units;
