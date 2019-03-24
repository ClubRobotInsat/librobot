#![no_std]
#![warn(missing_docs)]
#![deny(missing_debug_implementations)]
#![warn(unsafe_code)]
#![allow(dead_code)]

//! La librairie du club pour les µ-controlleurs arm.

#[cfg(test)]
#[macro_use]
extern crate std;

#[macro_use]
extern crate serde_derive;

pub use crate::transmission::*;

pub mod navigation;
pub mod transmission;
pub mod units;
