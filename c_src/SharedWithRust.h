//
// Created by terae on 8/25/18.
//

// Ce fichier regroupe des structures codées en C qui sont partagées avec l'électronique Rust
#ifndef ROOT_SHAREDWITHRUST_H
#define ROOT_SHAREDWITHRUST_H

//#include <stdint.h>
#include <inttypes.h>

#define MAX_SERVOS 8
#define MAX_CONTROLLED_MOTORS 8
#define MAX_UNCONTROLLED_MOTORS 8
#define MAX_BRUSHLESS 8

// Allows the header to be use from both C (so Rust) and C++
#ifdef __cplusplus
extern "C" {
#endif

const uint8_t NBR_SERVOS = MAX_SERVOS;
const uint8_t NBR_CONTROLLED_MOTOR = MAX_CONTROLLED_MOTORS;
const uint8_t NBR_UNCONTROLLED_MOTOR = MAX_UNCONTROLLED_MOTORS;
const uint8_t NBR_BRUSHLESS = MAX_BRUSHLESS;

/// Structure commune de l'ensemble des servos
typedef struct SharedServos2019 {
	/// Composition interne d'un servo-moteur
	struct Servo2019 {
		uint8_t id; // 'id == 0' veut dire qu'il n'y a pas de servo-moteur
		uint16_t position;
		// La commande est une vitesse OU un angle, le choix est fait depuis le booléen 'command'
		uint16_t command;
		uint8_t command_type; // si 0, la commande est en angle ; si 1, la commande est en vitesse
		char blocked;
		uint8_t blocking_mode;
		uint8_t color;
	} servos[MAX_SERVOS];

	uint8_t nb_servos;
	uint8_t parsing_failed;

} SharedServos2019;

typedef struct SharedMotors2019 {
	struct ControlledMotor2019 {
		uint8_t id; // `id == 0` veut dire qu'il n'y a pas de moteur
		uint8_t wanted_angle_position;
		uint8_t wanted_nb_turns;
		uint8_t finished;
		uint8_t new_command; // flag vrai si on a envoyé une consigne d'angle ou de tours
	} controlled_motors[MAX_CONTROLLED_MOTORS];

	struct UncontrolledMotor2019 {
		uint8_t id; // `id == 0` veut dire qu'il n'y a pas de moteur
		uint8_t on_off;
		uint8_t rotation;
	} uncontrolled_motors[MAX_UNCONTROLLED_MOTORS];

	struct Brushless2019 {
		uint8_t id; // `id == 0` veut dire qu'il n'y a pas de brushless
		uint8_t on_off;
	} brushless[MAX_BRUSHLESS];

	uint8_t parsing_failed;

} SharedMotors2019;

typedef struct SharedAvoidance2019 {
	int angle_detection_adversary;
	uint8_t adversary_detected;
	uint8_t parsing_failed;
} SharedAvoidance2019;

typedef struct SharedIO2019 {
	uint8_t tirette; // 1 correspond à 'tirette mise', 0 à 'tirette enlevée'
	uint8_t parsing_failed;
} SharedIO2019;

typedef struct SharedMoving2019 {
	// FIXME : en C, une est forcément de type 'int' (y penser pour les trames : cast)
	enum MoveType /*: uint8_t*/ {
		MOVE_NOTHING,
		MOVE_STOP = 0,
		MOVE_FORWARD = 1,
		MOVE_BACKWARD = 2,
		MOVE_TURN_RELATIVE = 3,
		MOVE_TURN_ABSOLUTE = 4,
		MOVE_FORWARD_INFINITY = 5,
		MOVE_BACKWARD_INFINITY = 6,
	} move_type;

	uint16_t pos_x;
	uint16_t pos_y;
	uint16_t angle;
	uint16_t linear_speed;
	uint16_t angular_speed;

	uint8_t reset; // Permets de définir de nouvelles coordonnées
	uint8_t blocked;
	uint8_t moving_done;
	uint8_t accuracy_reached;
	uint8_t servitude_on_off;
	uint8_t led;

	uint8_t parsing_failed;
} SharedMoving2019;

typedef struct SharedLED2019 {
	uint8_t on_off;
	uint8_t parsing_failed;
} SharedLED2019;

/// Fonctions définies en C et utilisées à la fois dans le code C++ et Rust

typedef uint8_t buffer_size_t;

// Format d'une trame :
// <nb_servo: u8>
// <[<id: u8> <position: u16> <command: u16> <command_type, blocking data, color: u8>] ...>
extern SharedServos2019 servo_read_frame(const uint8_t* message, buffer_size_t size);
extern buffer_size_t servo_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedServos2019* obj);
extern uint8_t get_size_servo_frame(uint8_t nb_servos);

// Format d'une trame :
// <nb_controlled: u8>
// <nb_uncontrolled: u8>
// <nb_brushless: u8>
// <[<id: u8> <wanted_angle: u8> <wanted_nb_turns: u8> <finished, new_command: u8>] ...>
// <[<id: u8> <on_off: u8>] ...>
// <[<id: u8> <on_off: u8>] ...>
extern SharedMotors2019 motor_read_frame(const uint8_t* message, buffer_size_t size);
extern buffer_size_t motor_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedMotors2019* obj);
extern uint8_t get_size_motor_frame(uint8_t nb_controlled, uint8_t nb_uncontrolled, uint8_t nb_brushless);

// TODO
extern SharedAvoidance2019 avoidance_read_frame(const uint8_t* message, buffer_size_t size);
extern buffer_size_t avoidance_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedAvoidance2019* obj);
extern uint8_t get_size_avoidance_frame();

// Format d'une trame :
// <tirette: uint8_t>
extern SharedIO2019 io_read_frame(const uint8_t* message, buffer_size_t size);
extern buffer_size_t io_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedIO2019* obj);
extern uint8_t get_size_io_frame();

// Format d'une trame :
// <on_off: uint8_t>
extern SharedLED2019 led_read_frame(const uint8_t* message, buffer_size_t size);
extern buffer_size_t led_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedLED2019* obj);
extern uint8_t get_size_led_frame();

// TODO
extern SharedMoving2019 moving_read_frame(const uint8_t* message, buffer_size_t size);
extern buffer_size_t moving_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedMoving2019* obj);
extern uint8_t get_size_moving_frame();

#ifdef __cplusplus
}
#endif

#endif // ROOT_SHAREDWITHRUST_H
