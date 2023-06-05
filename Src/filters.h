/*
 * filters.h
 *
 *  Created on: 24.09.2021
 *      Author: symon
 */

#ifndef FILTERS_H_
#define FILTERS_H_

#include <stdint.h>
#include "global_constants.h"

// "Notch filter is just a combination of low and high pass filter" - not really! This description is good for band-stop filter
// Notch filter is much more precise and narrower. It is a combination of:
// - 2nd order system (inversion of it) with damping = 0, which define center frequency,
// - 2 poles away from the center frequency which define width of a filter.
// But still notch filter, as well as a band-stop, is just an IIR filter with suitable coefficients.

// in general notch filter can be higher order but it can be also done by serial connecting a few notch filters 2nd order.

// in case of 2nd order filters circle buffer has no sense so lets create special struct for 2nd order filters:

typedef struct
{
	float frequency; // 	cut-off/center frequency
	float Q_factor;	 //	quality factor
	float a0;
	float a1;
	float a2;
	float b0;
	float b1;
	float b2;
	float x1; //	last input
	float x2; //	2nd last input
	float y1; //	last output
	float y2; //	2nd last output
} biquad_Filter_t;

typedef enum
{
	BIQUAD_LPF,
	BIQUAD_NOTCH,
	BIQUAD_BPF,
} biquad_Filter_type;

typedef struct
{
	biquad_Filter_t notch_filters[3][MOTORS_COUNT][RPM_MAX_HARMONICS]; // each axes (X,Y,Z) for each motor and its harmonics
	float weight[3][MOTORS_COUNT][RPM_MAX_HARMONICS];				   // weight used to fade out filter (0 - filter is off, 1 - is used in 100%)
	float q_factor;													   // q_factor for all notches
	uint8_t harmonics;												   // number of filtered harmonics

} RPM_filter_t;

void biquad_filter_init(biquad_Filter_t *filter, biquad_Filter_type filter_type, float center_frequency_Hz, float quality_factor, uint16_t sampling_frequency_Hz);
float biquad_filter_apply_DF1(biquad_Filter_t *filter, float input);
float biquad_filter_apply_DF2(biquad_Filter_t *filter, float input);

void RPM_filter_init(RPM_filter_t *filter, uint16_t sampling_frequency_Hz);
float RPM_filter_apply(RPM_filter_t *filter, uint8_t axis, float input);
void RPM_filter_update(RPM_filter_t *filter);

#endif /* FILTERS_H_ */
