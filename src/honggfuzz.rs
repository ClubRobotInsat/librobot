#[macro_use]
extern crate honggfuzz;
extern crate librobot;

use librobot::transmission::Message;
use librobot::transmission::*;
use std::cmp::min;

fn main() {
    loop {
        fuzz!(|data: &[u8]| {
            let mut msg = Message::new();
            for b in &data[0..min(data.len(), msg.capacity()) as usize] {
                msg.push(*b);
            }
            let _ = ServoGroup::new(msg);
        });
    }
}
