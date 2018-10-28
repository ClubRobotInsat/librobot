#![no_main]
#[macro_use] extern crate libfuzzer_sys;
extern crate librobot;

use std::cmp::min;
use librobot::transmission::*;

fuzz_target!(|data: &[u8]| {
    let mut msg = Message::new();
    for b in &data[0..min(data.len(), msg.capacity()) as usize] {
        msg.push(*b);
    }
    let _ = ServoGroup::new(msg);
});
