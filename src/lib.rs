#![no_std]
#![warn(missing_docs)]
#![deny(missing_debug_implementations)]
#![deny(unsafe_code)]

//! Librairie du club

extern crate embedded_hal;

pub mod trame;
pub mod utils;

pub use trame::Trame;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
