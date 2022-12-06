/*
 * global_constants.h
 *
 *  Created on: 18.04.2021
 *      Author: symon
 */

#ifndef GLOBAL_CONSTANTS_H_
#define GLOBAL_CONSTANTS_H_
#include <stdbool.h>

//------------ESC_PROTOCOLS----------
#define BIT_BANGING_V2
#define DSHOT_MODE 300 // 150 300 600 1200

#define DSHOT_BUFFER_LENGTH 18 // 16 bits of Dshot and 2 for clearing
#define DSHOT_PWM_FRAME_LENGTH 35
#define DSHOT_1_LENGTH 26
#define DSHOT_0_LENGTH 13

#if defined(BIT_BANGING_V1)
#define DSHOT_BB_BUFFER_LENGTH 18  // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
#define DSHOT_BB_FRAME_LENGTH 140  // how many counts of timer gives one bit frame
#define DSHOT_BB_FRAME_SECTIONS 14 // in how many sections is bit frame divided (must be factor of DSHOT_BB_FRAME_LENGTH)
#define DSHOT_BB_1_LENGTH 10
#define DSHOT_BB_0_LENGTH 4

#elif defined(BIT_BANGING_V2)
#define DSHOT_BB_BUFFER_LENGTH 18 // 16 bits of Dshot and 2 for clearing - used when bit-banging dshot used
#define DSHOT_BB_FRAME_LENGTH 35
#define DSHOT_BB_1_LENGTH 26
#define DSHOT_BB_0_LENGTH 13
#define DSHOT_BB_FRAME_SECTIONS 3
#endif

#define BDSHOT_RESPONSE_LENGTH 21
#define BDSHOT_RESPONSE_BITRATE (DSHOT_MODE * 4 / 3) // in my tests this value was not 5/4 * DSHOT_MODE as documentation suggests
#define BDSHOT_RESPONSE_OVERSAMPLING 3               // it has to be a factor of (DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE)

//----------MOTORS---------
#define MOTORS_COUNT 4        // how many motors are used
#define MOTOR_1 3             // PA3
#define MOTOR_2 0             // PB0
#define MOTOR_3 1             // PB1
#define MOTOR_4 2             // PA2
#define MOTOR_POLES_NUMBER 14 // how many poles have your motors (usually 14 or 12)

//-------------------FILTERS------------------
#define FREQUENCY_OF_SAMPLING_HZ 900                                 // frequency of sampling data [Hz]
#define MAX_FREQUENCY_FOR_FILTERING 0.48f * FREQUENCY_OF_SAMPLING_HZ // maximal frequency that can be filtered (almost Nyquist frequency) [Hz]

#define BIQUAD_LPF_CUTOFF 50        //	[Hz]
#define BIQUAD_LPF_Q 1.f / sqrtf(2) //	Q factor for Low-pass filters
#define BIQUAD_NOTCH_Q 100          //	Q factor for notch filters (bigger value -> narrower notch)

// RPM filter:
#define USE_RPM_FILTER // if you want use RPM_filter USE_RPM_FILTER

#define RPM_MIN_FREQUENCY_HZ 50 // all frequencies <= than this value will not be filtered by RPM filter
#define RPM_FADE_RANGE_HZ 50    // fade out notch when approaching RPM_MIN_FREQUENCY_HZ (turn it off for RPM_MIN_FREQUENCY_HZ)
#define RPM_Q_FACTOR 500        // Q factor for all notches. It is VERY HIGH therefore notches are really narrow and selective
#define RPM_MAX_HARMONICS 3     // max. number of filtered harmonics

#endif /*GLOBAL_CONSTANTS_H_*/