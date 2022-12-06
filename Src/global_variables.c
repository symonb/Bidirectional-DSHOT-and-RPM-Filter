/*
 * global_variables.c
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"

// motor's values set by PID's:
uint16_t motor_1_value;
uint16_t motor_2_value;
uint16_t motor_3_value;
uint16_t motor_4_value;

//	motor's RPM values (from BDshot)
uint32_t motors_rpm[MOTORS_COUNT];

// used in BDshot:
float motors_error[MOTORS_COUNT];

// pointers for motor's values:
uint16_t *motor_1_value_pointer;
uint16_t *motor_2_value_pointer;
uint16_t *motor_3_value_pointer;
uint16_t *motor_4_value_pointer;

uint32_t dshot_bb_buffer_1_4[DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS];
uint32_t dshot_bb_buffer_2_3[DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS];
// BDSHOT response is being sampled just after transmission. There is ~33 [us] break before response (additional sampling) and bitrate is increased by 5/4:
uint32_t dshot_bb_buffer_1_4_r[(int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
uint32_t dshot_bb_buffer_2_3_r[(int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING];
