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
extern crate libc;
extern crate embedded_hal;
extern crate nb;

#[macro_use]
pub mod trame;
pub mod trame_reader;
pub mod utils;
pub mod c_struct;

pub use trame::Trame;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
