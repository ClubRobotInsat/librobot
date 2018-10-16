//
// Created by terae on 8/25/18.
//

#include "SharedWithRust.h"
#include <stddef.h>

#define FRAME_SERVO_SIZE(number) (1 + (number)*6)
#define FRAME_MOTOR_SIZE(controlled, uncontrolled, brushless) (3 + (controlled)*4 + (uncontrolled)*2 + (brushless)*2)

SharedServos2019 servo_read_frame(const uint8_t* message, buffer_size_t size) {
	SharedServos2019 s;

	// Contrôle de la taille du flux d'octet reçu
	if(message == NULL || size == 0) {
		s.parsing_failed = 1;
		return s;
	}
	uint8_t count = 0;
	uint8_t nb_servo = message[count++];

	if(size != FRAME_SERVO_SIZE(nb_servo)) {
		s.parsing_failed = 1;
		return s;
	}

	// Initialisation de la struct
	for(uint8_t index = 0; index < MAX_SERVOS; ++index) {
		s.servos[index].id = 0;
	}

	// Parsing de chaque servo-moteur
	for(uint8_t index = 0; index < nb_servo; ++index) {
		uint8_t id = message[count++];

		// Contrôle de l'ID
		if(id == 0) {
			s.parsing_failed = 1;
			return s;
		}
		for (uint8_t j = 0; j < MAX_SERVOS; ++j) {
			// L'ID doit être unique
			if (s.servos[j].id == id) {
				s.parsing_failed = 1;
				return s;
			}
		}

		// Reconstruction du servo-moteur `id`
		s.servos[index].id = id;

		uint16_t position = message[count++];
		position <<= 8;
		position |= message[count++];
		s.servos[index].position = position;

		uint16_t command = message[count++];
		command <<= 8;
		command |= message[count++];
		s.servos[index].command = command;

		uint8_t infos = message[count++];
		s.servos[index].command_type = (uint8_t)((0b00100000 & infos) >> 5);
		s.servos[index].blocked = (char)((0b00010000 & infos) >> 4);
		s.servos[index].blocking_mode = (uint8_t)((0b00001000 & infos) >> 3);
		s.servos[index].color = (uint8_t)(0b00000111 & infos);
	}

	// Tout s'est bien passé
	s.parsing_failed = 0;
	s.nb_servos = nb_servo;
	return s;
}

buffer_size_t servo_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedServos2019* obj) {
    buffer_size_t size = 0;
	uint8_t nb_servo = 0;

	// Contrôle du buffer alloué
	if(buf == NULL || obj == NULL || buf_size == 0) {
		return 0;
	}

	for(uint8_t id = 0; id < MAX_SERVOS; ++id) {
		if(obj->servos[id].id > 0) {
			++nb_servo;
		}
	}

	// Il n'y a pas assez de place dans le buffer : on n'écrit rien dedans
	if(buf_size < FRAME_SERVO_SIZE(nb_servo)) {
		return 0;
	}

	// Écriture des données de chaque servo-moteur
	buf[size++] = nb_servo;
	for(uint8_t index = 0; index < obj->nb_servos; ++index) {
		// Le servo-moteur d'index
		if(obj->servos[index].id > 0) {
			buf[size++] = obj->servos[index].id;

			buf[size++] = (uint8_t)((UINT8_MAX - ((0xff00 & obj->servos[index].position) >> 8)) ^ UINT8_MAX);
			buf[size++] = (uint8_t)((UINT8_MAX - obj->servos[index].position) ^ UINT8_MAX);

			buf[size++] = (uint8_t)((UINT8_MAX - ((0xff00 & obj->servos[index].command) >> 8)) ^ UINT8_MAX);
			buf[size++] = (uint8_t)((UINT8_MAX - obj->servos[index].command) ^ UINT8_MAX);

			// Format de l'octet de données : [0b76543210]
			// index '210' : couleur
			// index '3' : blocking_mode
			// index '4' : blocked
			// index '5' : command_type
			uint8_t infos = obj->servos[index].command_type;
			infos <<= 1;
			infos |= obj->servos[index].blocked;
			infos <<= 1;
			infos |= obj->servos[index].blocking_mode;
			infos <<= 3;
			infos |= obj->servos[index].color;
			buf[size++] = infos;
		}
	}

	return size;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SharedMotors2019 motor_read_frame(const uint8_t* message, buffer_size_t size) {
	SharedMotors2019 s;

	if(message == NULL || size < 3) {
		s.parsing_failed = 1;
		return s;
	}
	uint8_t count = 0;
	uint8_t nb_controlled = message[count++];
	uint8_t nb_uncontrolled = message[count++];
	uint8_t nb_brushless = message[count++];

	if(size != FRAME_MOTOR_SIZE(nb_controlled, nb_uncontrolled, nb_brushless)) {
		s.parsing_failed = 1;
		return s;
	}

	for(uint8_t i = 0; i < MAX_CONTROLLED_MOTORS; ++i) {
		s.controlled_motors[i].id = 0;
	}

	for(uint8_t i = 0; i < MAX_UNCONTROLLED_MOTORS; ++i) {
		s.uncontrolled_motors[i].id = 0;
	}

	for(uint8_t i = 0; i < MAX_BRUSHLESS; ++i) {
		s.brushless[i].id = 0;
	}

	for(uint8_t index = 0; index < nb_controlled; ++index) {
		uint8_t id = message[count++];

		if(id == 0) {
			s.parsing_failed = 1;
			return s;
		}

		s.controlled_motors[index].id = id;
		s.controlled_motors[index].wanted_angle_position = message[count++];
		s.controlled_motors[index].wanted_nb_turns = message[count++];

		uint8_t infos = message[count++];
		s.controlled_motors[index].finished = (uint8_t)((0b00000010 & infos) >> 1);
		s.controlled_motors[index].new_command = (uint8_t)(0b00000001 & infos);
	}

	for(uint8_t index = 0; index < nb_uncontrolled; ++index) {
		uint8_t id = message[count++];

		if(id == 0) {
			s.parsing_failed = 1;
			return s;
		}

		s.uncontrolled_motors[index].id = id;
		uint8_t infos = message[count++];
		s.uncontrolled_motors[index].on_off = (uint8_t)((0b00000010 & infos) >> 1);
		s.uncontrolled_motors[index].rotation = (uint8_t)(0b00000001 & infos);
	}

	for(uint8_t index = 0; index < nb_brushless; ++index) {
		uint8_t id = message[count++];

		if(id == 0) {
			s.parsing_failed = 1;
			return s;
		}

		s.brushless[index].id = id;
		s.brushless[index].on_off = message[count++];
	}

	s.parsing_failed = 0;
	return s;
}

buffer_size_t motor_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedMotors2019* obj) {
    buffer_size_t size = 0;
	uint8_t nb_controlled = 0;
	uint8_t nb_uncontrolled = 0;
	uint8_t nb_brushless = 0;

	if(buf == NULL || obj == NULL || buf_size == 0) {
		return 0;
	}

	// Compte de chaque type de moteurs
	for(uint8_t index = 0; index < MAX_CONTROLLED_MOTORS; ++index) {
		if(obj->controlled_motors[index].id > 0) {
			++nb_controlled;
		}
	}

	for(uint8_t index = 0; index < MAX_UNCONTROLLED_MOTORS; ++index) {
		if(obj->uncontrolled_motors[index].id > 0) {
			++nb_uncontrolled;
		}
	}

	for(uint8_t index = 0; index < MAX_BRUSHLESS; ++index) {
		if(obj->brushless[index].id > 0) {
			++nb_brushless;
		}
	}

	// Il n'y a pas assez de place dans le buffer : on n'écrit rien dedans
	if(buf_size < FRAME_MOTOR_SIZE(nb_controlled, nb_uncontrolled, nb_brushless)) {
		return 0;
	}

	// Ecriture de chaque type de moteur
	buf[size++] = nb_controlled;
	buf[size++] = nb_uncontrolled;
	buf[size++] = nb_brushless;

	for(uint8_t index = 0; index < MAX_CONTROLLED_MOTORS; ++index) {
		if(obj->controlled_motors[index].id > 0) {
			buf[size++] = obj->controlled_motors[index].id;

			buf[size++] = obj->controlled_motors[index].wanted_angle_position;
			buf[size++] = obj->controlled_motors[index].wanted_nb_turns;

			uint8_t infos = obj->controlled_motors[index].finished;
			infos <<= 1;
			infos |= obj->controlled_motors[index].new_command;
			buf[size++] = infos;
		}
	}

	for(uint8_t index = 0; index < MAX_UNCONTROLLED_MOTORS; ++index) {
		if(obj->uncontrolled_motors[index].id > 0) {
			buf[size++] = obj->uncontrolled_motors[index].id;

			uint8_t infos = obj->uncontrolled_motors[index].on_off;
			infos <<= 1;
			infos |= obj->uncontrolled_motors[index].rotation;
			buf[size++] = infos;
		}
	}

	for(uint8_t index = 0; index < MAX_BRUSHLESS; ++index) {
		if(obj->brushless[index].id > 0) {
			buf[size++] = obj->brushless[index].id;

			buf[size++] = obj->brushless[index].on_off;
		}
	}

	return size;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SharedAvoidance2019 avoidance_read_frame(const uint8_t* message, buffer_size_t size) {
	SharedAvoidance2019 s;
	s.parsing_failed = 1;
	return s;
}

buffer_size_t avoidance_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedAvoidance2019* obj) {
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SharedIO2019 io_read_frame(const uint8_t* message, buffer_size_t size) {
	SharedIO2019 s;

	if(message == NULL || size == 0) {
		s.parsing_failed = 1;
		return s;
	}

	s.tirette = message[0];
	s.parsing_failed = 0;

	return s;
}

buffer_size_t io_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedIO2019* obj) {
	if(buf == NULL || obj == NULL || buf_size == 0) {
		return 0;
	}

	buf[0] = obj->tirette;
	return 1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SharedMoving2019 moving_read_frame(const uint8_t* message, buffer_size_t size) {
	SharedMoving2019 s;
	s.parsing_failed = 1;
	return s;
}

buffer_size_t moving_write_frame(uint8_t* buf, buffer_size_t buf_size, const SharedMoving2019* obj) {
	return 0;
}
