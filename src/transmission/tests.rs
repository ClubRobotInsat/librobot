//#![no_std]
extern crate core;
extern crate libc;

use arrayvec::ArrayVec;
use transmission::ffi::*;
use transmission::servos::*;

#[test]
fn test_servos() {
    let servo_empty = CServo2019 {
        id: 0,
        position: 0,
        command: 0,
        command_type: 0,
        blocked: 0,
        blocking_mode: 0,
        color: 0,
    };
    assert_eq!(
        servo_empty,
        CServo2019 {
            id: 0,
            position: 1,
            command: 2,
            command_type: 1,
            blocked: 4,
            blocking_mode: 5,
            color: 6,
        }
    );
    let servo1 = CServo2019 {
        id: 1,
        position: 512,
        command: 162,
        command_type: 1,
        blocked: 0,
        blocking_mode: 0,
        color: 5,
    };
    assert_eq!(servo1, servo1);
    let servo3 = CServo2019 {
        id: 3,
        position: 1000,
        command: 10,
        command_type: 1,
        blocked: 1,
        blocking_mode: 1,
        color: 3,
    };
    assert_ne!(servo1, servo3);

    let mut array = [servo_empty; 8];
    array[0] = servo1;
    array[1] = servo3;

    let struct_before = CSharedServos2019 {
        servos: array,
        parsing_failed: 0,
        nb_servos: 2,
    };

    let written_frame = struct_before.write_frame();
    assert!(written_frame.is_ok());
    let read_frame =
        CSharedServos2019::read_frame(written_frame.unwrap_or(ArrayVec::<[u8; 256]>::new()));
    assert!(read_frame.is_ok());
    let struct_after = read_frame.unwrap_or(struct_before);
    // Les éléments ne sont pas dans le même ordre mais les structures sont équivalentes
    for servo in &struct_before.servos {
        assert!(
            &struct_after
                .servos
                .iter()
                .find(|elem| *elem == servo)
                .is_some()
        );
    }

    assert_eq!(ServoGroup::get_size_frame(5), 31);
}

#[test]
fn test_motors() {
    let controlled_empty = CControlledMotor2019 {
        id: 0,
        wanted_angle_position: 0,
        wanted_nb_turns: 0,
        finished: 0,
        new_command: 0,
    };
    assert_eq!(
        controlled_empty,
        CControlledMotor2019 {
            id: 0,
            wanted_angle_position: 1,
            wanted_nb_turns: 2,
            finished: 3,
            new_command: 4,
        }
    );
    let controlled_1 = CControlledMotor2019 {
        id: 1,
        wanted_angle_position: 213,
        wanted_nb_turns: 2,
        finished: 0,
        new_command: 1,
    };
    assert_eq!(controlled_1, controlled_1);
    let controlled_3 = CControlledMotor2019 {
        id: 3,
        wanted_angle_position: 12,
        wanted_nb_turns: 5,
        finished: 1,
        new_command: 0,
    };
    assert_ne!(controlled_1, controlled_3);

    let uncontrolled_empty = CUncontrolledMotor2019 {
        id: 0,
        on_off: 0,
        rotation: 0,
    };
    assert_eq!(
        uncontrolled_empty,
        CUncontrolledMotor2019 {
            id: 0,
            on_off: 1,
            rotation: 0,
        }
    );
    let uncontrolled_1 = CUncontrolledMotor2019 {
        id: 1,
        on_off: 0,
        rotation: 0,
    };
    assert_eq!(uncontrolled_1, uncontrolled_1);

    let brushless_empty = CBrushless2019 { id: 0, on_off: 0 };
    assert_eq!(brushless_empty, CBrushless2019 { id: 0, on_off: 1 });
    let brushless_1 = CBrushless2019 { id: 1, on_off: 0 };
    assert_eq!(brushless_1, brushless_1);
    let brushless_5 = CBrushless2019 { id: 5, on_off: 0 };
    let brushless_6 = CBrushless2019 { id: 6, on_off: 1 };

    let mut array_controlled = [controlled_empty; 8];
    array_controlled[1] = controlled_1;
    array_controlled[3] = controlled_3;

    let mut array_uncontrolled = [uncontrolled_empty; 8];
    array_uncontrolled[1] = uncontrolled_1;

    let mut array_brushless = [brushless_empty; 8];
    array_brushless[1] = brushless_1;
    array_brushless[5] = brushless_5;
    array_brushless[6] = brushless_6;

    let struct_before = CSharedMotors2019 {
        controlled_motors: array_controlled,
        uncontrolled_motors: array_uncontrolled,
        brushless: array_brushless,

        parsing_failed: 0,
    };

    let written_frame = struct_before.write_frame();
    assert!(written_frame.is_ok());
    let read_frame =
        CSharedMotors2019::read_frame(written_frame.unwrap_or(ArrayVec::<[u8; 256]>::new()));
    assert!(read_frame.is_ok());
    let struct_after = read_frame.unwrap_or(struct_before);

    for motor in &struct_before.controlled_motors {
        assert!(
            &struct_after
                .controlled_motors
                .iter()
                .find(|elem| *elem == motor)
                .is_some()
        );
    }
    for motor in &struct_before.uncontrolled_motors {
        assert!(
            &struct_after
                .uncontrolled_motors
                .iter()
                .find(|elem| *elem == motor)
                .is_some()
        );
    }
    for motor in &struct_before.brushless {
        assert!(
            &struct_after
                .brushless
                .iter()
                .find(|elem| *elem == motor)
                .is_some()
        );
    }
}
