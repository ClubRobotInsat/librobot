#![no_std]
#![warn(missing_docs)]
#![deny(missing_debug_implementations)]
#![deny(unsafe_code)]
#![allow(dead_code)]

//! Librairie du club

extern crate embedded_hal;
extern crate arrayvec;
extern crate nb;

pub mod trame;
pub mod utils;
pub mod trame_reader;

pub use trame::Trame;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
