/*
 * global_variables.h
 *
 *  Created on: 05.05.2021
 *      Author: symon
 */

#ifndef GLOBAL_VARIABLES_H_
#define GLOBAL_VARIABLES_H_

#include "stdint.h"
#include "stdbool.h"

extern uint32_t motors_rpm[];

extern float motors_error[];

extern uint16_t *motor_1_value_pointer;
extern uint16_t *motor_2_value_pointer;
extern uint16_t *motor_3_value_pointer;
extern uint16_t *motor_4_value_pointer;

extern uint32_t dshot_bb_buffer_1_4[];
extern uint32_t dshot_bb_buffer_2_3[];
extern uint32_t dshot_bb_buffer_1_4_r[];
extern uint32_t dshot_bb_buffer_2_3_r[];

#endif /* GLOBAL_VARIABLES_H_ */
