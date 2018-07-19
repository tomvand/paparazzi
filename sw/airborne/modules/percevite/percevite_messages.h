#ifndef PERCEVITE_MESSAGES_H
#define PERCEVITE_MESSAGES_H

#include <stdint.h>

enum slamdunk_flags_t {
	SD_MSG_FLAG_SAFE_DISTANCE = 0x01,
	SD_MSG_FLAG_VELOCITY      = 0x02,
};

union slamdunk_to_paparazzi_msg_t {
	struct {
		uint8_t flags;         // Indicate which fields are set in the message
		uint8_t safe_distance; // [dm]
		uint8_t valid_pixels;  // [fraction 0-255]
		float vx;              // [m/s] Front-right-down velocities (in drone frame)
		float vy;
		float vz;
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union slamdunk_to_paparazzi_msg_t SlamdunkToPaparazziMsg;

union paparazzi_to_slamdunk_msg_t {
	struct {
		char text[20]; // Dummy payload
	};
	unsigned char bytes;
} __attribute((__packed__));
typedef union paparazzi_to_slamdunk_msg_t PaparazziToSlamdunkMsg;

#endif