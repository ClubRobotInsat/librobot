#[macro_use]
extern crate librobot;
extern crate arrayvec;

use librobot::transmission::*;

use arrayvec::ArrayVec;

use std::fs::File;
use std::io::Write;

fn main() {
    let outfiles = "msg1";
    let mut tab = [Servo::default(); 8];
    tab[1] = Servo {
        id: 89,
        known_position: 25,
        control: Control::Speed(56),
        blocked: false,
        mode: BlockingMode::Unblocking,
        color: Color::Black,
    };
    tab[2] = Servo {
        id: 0,
        known_position: 1023,
        control: Control::Speed(80),
        blocked: true,
        mode: BlockingMode::Unblocking,
        color: Color::Red,
    };
    tab[3] = Servo {
        id: 255,
        known_position: 512,
        control: Control::Position(12),
        blocked: false,
        mode: BlockingMode::HoldOnblock,
        color: Color::Green,
    };
    tab[4] = Servo {
        id: 254,
        known_position: 1,
        control: Control::Speed(1023),
        blocked: false,
        mode: BlockingMode::Unblocking,
        color: Color::Red,
    };
    tab[5] = Servo {
        id: 127,
        known_position: 999,
        control: Control::Position(1023),
        blocked: true,
        mode: BlockingMode::HoldOnblock,
        color: Color::Magenta,
    };
    let mut vec: ArrayVec<[Servo; 8]> = ArrayVec::new();
    for elem in tab.into_iter() {
        vec.push(*elem);
    }
    let servos = ServoGroup { servos: vec };

    let bytes = servos.into_bytes().unwrap();
    let mut file = File::create(outfiles).unwrap();
    file.write(&bytes).unwrap();
}
