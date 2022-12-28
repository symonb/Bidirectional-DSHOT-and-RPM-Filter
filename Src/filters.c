/*
 * filters.c
 *
 *  Created on: 24.09.2021
 *      Author: symon
 */
#include "stm32f4xx.h"
#include <math.h>
#include "global_constants.h"
#include "global_variables.h"
#include "filters.h"

static void biquad_filter_update(biquad_Filter_t *filter, biquad_Filter_type filter_type, float filter_frequency_Hz, float quality_factor, uint16_t sampling_frequency_Hz);
static void biquad_filter_copy_coefficients(biquad_Filter_t *copy_from_filter, biquad_Filter_t *copy_to_filter);
static void RPM_filter_update(RPM_filter_t *filter);

void biquad_filter_init(biquad_Filter_t *filter, biquad_Filter_type filter_type, float filter_frequency_Hz, float quality_factor, uint16_t sampling_frequency_Hz)
{
	biquad_filter_update(filter, filter_type, filter_frequency_Hz, quality_factor, sampling_frequency_Hz);

	// set previous values as 0:
	filter->x1 = 0;
	filter->x2 = 0;
	filter->y1 = 0;
	filter->y2 = 0;
}

static void biquad_filter_update(biquad_Filter_t *filter, biquad_Filter_type filter_type, float filter_frequency_Hz, float quality_factor, uint16_t sampling_frequency_Hz)
{
	// save filter info:
	filter->frequency = filter_frequency_Hz;
	filter->Q_factor = quality_factor;
	const float omega = 2.f * M_PI * filter_frequency_Hz / sampling_frequency_Hz;
	const float sn = sinf(omega);
	const float cs = cosf(omega);
	const float alpha = sn / (2.0f * filter->Q_factor);

	// implementation from datasheet:https://www.ti.com/lit/an/slaa447/slaa447.pdf <-- not everything good
	// or even better: http://shepazu.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html <-- probably resource for above

	/*
	general formula for 2nd order filter:

			   b0*z^-2 + b1*z^-1 + b2
	H(z^-1) = ------------------------
			   a0*z^-2 + a1*z^-1 + a2

	main analog prototypes:

	Low-pass:
				 1
	H(s) = ---------------
			s^2 + s/Q + 1

	Notch:
			   s^2 + 1
	H(s) = ---------------
			s^2 + s/Q + 1

	BPF (skirt gain where Q is max. gain):
				 s
	H(s) = ---------------
			s^2 + s/Q + 1

	Q is quality factor
	In this datasheet conversion from s -> z domain is done with Bilinear transform with pre-warping
	*/

	filter->a0 = 1 + alpha;
	switch (filter_type)
	{
	case BIQUAD_LPF:

		filter->b0 = (1 - cs) * 0.5f;
		filter->b1 = 1 - cs;
		filter->b2 = filter->b0;
		filter->a1 = -2 * cs;
		filter->a2 = 1 - alpha;
		break;
	case BIQUAD_NOTCH:
		filter->b0 = 1;
		filter->b1 = -2 * cs;
		filter->b2 = 1;
		filter->a1 = filter->b1;
		filter->a2 = 1 - alpha;
		break;
	case BIQUAD_BPF:
		filter->b0 = alpha;
		filter->b1 = 0;
		filter->b2 = -alpha;
		filter->a1 = -2 * cs;
		filter->a2 = 1 - alpha;
		break;
	}

	// oust a0 coefficient:
	filter->b0 /= filter->a0;
	filter->b1 /= filter->a0;
	filter->b2 /= filter->a0;
	filter->a1 /= filter->a0;
	filter->a2 /= filter->a0;
}

float biquad_filter_apply_DF1(biquad_Filter_t *filter, float input)
{
	// compute result:
	const float result = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;

	// shift x1 to x2, input to x1:
	filter->x2 = filter->x1;
	filter->x1 = input;

	// shift y1 to y2, result to y1:
	filter->y2 = filter->y1;
	filter->y1 = result;

	return result;
}

float biquad_filter_apply_DF2(biquad_Filter_t *filter, float input)
{
	//	this is transposed direct form 2 is a little more precised for float number implementation:

	const float result = filter->b0 * input + filter->x1;

	filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
	filter->x2 = filter->b2 * input - filter->a2 * result;

	return result;
}

static void biquad_filter_copy_coefficients(biquad_Filter_t *copy_from_filter, biquad_Filter_t *copy_to_filter)
{

	copy_to_filter->a0 = copy_from_filter->a0;
	copy_to_filter->a1 = copy_from_filter->a1;
	copy_to_filter->a2 = copy_from_filter->a2;

	copy_to_filter->b0 = copy_from_filter->b0;
	copy_to_filter->b1 = copy_from_filter->b1;
	copy_to_filter->b2 = copy_from_filter->b2;
}

void RPM_filter_init(RPM_filter_t *filter, uint16_t sampling_frequency_Hz)
{
	filter->harmonics = RPM_MAX_HARMONICS;
	filter->q_factor = RPM_Q_FACTOR;
	const float default_freq = 100; // only for initialization doesn't really matter

	// initialize notch filters:
	for (uint8_t axis = 0; axis < 3; axis++)
	{
		for (uint8_t motor = 0; motor < MOTORS_COUNT; motor++)
		{
			for (uint8_t harmonic = 0; harmonic < RPM_MAX_HARMONICS; harmonic++)
			{
				biquad_filter_init(&(filter->notch_filters[axis][motor][harmonic]), BIQUAD_NOTCH, default_freq, filter->q_factor, sampling_frequency_Hz);
				filter->weight[axis][motor][harmonic] = 1;
			}
		}
	}
}

static void RPM_filter_update(RPM_filter_t *filter)
{
	const uint8_t sec_in_min = 60; // for conversion from Hz to rpm
	float frequency;			   // frequency for filtering
	// each motor introduces its own frequency (with harmonics) but for every axes noises are the same;

	for (uint8_t motor = 0; motor < MOTORS_COUNT; motor++)
	{
		for (uint8_t harmonic = 0; harmonic < RPM_MAX_HARMONICS; harmonic++)
		{
			frequency = (float)motors_rpm[motor] * (harmonic + 1) / sec_in_min;
			if (frequency > RPM_MIN_FREQUENCY_HZ)
			{
				if (frequency < MAX_FREQUENCY_FOR_FILTERING)
				{
					// each axis has the same noises from motors, so compute it once and next copy values:
					biquad_filter_update(&(filter->notch_filters[0][motor][harmonic]), BIQUAD_NOTCH, frequency, filter->q_factor, FREQUENCY_OF_SAMPLING_HZ);
					biquad_filter_copy_coefficients(&(filter->notch_filters[0][motor][harmonic]), &(filter->notch_filters[1][motor][harmonic]));
					biquad_filter_copy_coefficients(&(filter->notch_filters[0][motor][harmonic]), &(filter->notch_filters[2][motor][harmonic]));

					// fade out if reaching minimal frequency:
					if (frequency < (RPM_MIN_FREQUENCY_HZ + RPM_FADE_RANGE_HZ))
					{
						filter->weight[0][motor][harmonic] = (float)(frequency - RPM_MIN_FREQUENCY_HZ) / (RPM_FADE_RANGE_HZ);
						filter->weight[1][motor][harmonic] = filter->weight[0][motor][harmonic];
						filter->weight[2][motor][harmonic] = filter->weight[0][motor][harmonic];
					}
					else
					{
						filter->weight[0][motor][harmonic] = 1;
						filter->weight[1][motor][harmonic] = 1;
						filter->weight[2][motor][harmonic] = 1;
					}
				}
				else
				{
					filter->weight[0][motor][harmonic] = 0;
					filter->weight[1][motor][harmonic] = 0;
					filter->weight[2][motor][harmonic] = 0;
				}
			}
			else
			{
				frequency = RPM_MIN_FREQUENCY_HZ;

				filter->weight[0][motor][harmonic] = 0;
				filter->weight[1][motor][harmonic] = 0;
				filter->weight[2][motor][harmonic] = 0;
			}
		}
	}
}

float RPM_filter_apply(RPM_filter_t *filter, uint8_t axis, float input)
{
	float result = input;

	for (uint8_t motor = 0; motor < MOTORS_COUNT; motor++)
	{
		for (uint8_t harmonic = 0; harmonic < RPM_MAX_HARMONICS; harmonic++)
		{
			result = filter->weight[axis][motor][harmonic] * biquad_filter_apply_DF1(&(filter->notch_filters[axis][motor][harmonic]), result) + (1 - filter->weight[axis][motor][harmonic]) * result;
		}
	}

	return result;
}
