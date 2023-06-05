
#include "stm32f4xx.h"
#include "filters.h"
#include "bdshot.h"
#include "setup.h"
#include "global_variables.h"

// This is only a sketch how to use RPM filters
// You should take care of time management and updating samples
// Below main() code allow to spin motors up to 50% of max speed and then decelerate, wait 3 sec and do it again.
// To do this some extra code (all below) is provided to take care of timing.
// For your code, this is not essential.

static void setup_TIM5();
static uint64_t global_time;
uint64_t get_global_time();
void delay_micro(uint16_t delay_time);

int main()
{

    float gyro_measurements[3];   //  tab for measurements that you want filter
    RPM_filter_t rpm_filter_gyro; //  RPM filter object for each sensor (with 3 axes measurements)

    // motor's values set by your PID's or by hand:
    uint16_t motor_1_value;
    uint16_t motor_2_value;
    uint16_t motor_3_value;
    uint16_t motor_4_value;

    // assign your variable addresses into pointers:
    motor_1_value_pointer = &motor_1_value;
    motor_2_value_pointer = &motor_2_value;
    motor_3_value_pointer = &motor_3_value;
    motor_4_value_pointer = &motor_4_value;

    setup();                                                     // setup everything
    setup_NVIC();                                                // enable NVIC
    setup_TIM5();                                                // setup for timining
    preset_bb_BDshot_buffers();                                  // preset buffers (do it once)
    RPM_filter_init(&rpm_filter_gyro, FREQUENCY_OF_SAMPLING_HZ); // initialize RPM filters (each filter need to be init)

    while (1)
    {
        /*
            here should be some kind of scheduler to provide expected frequencies
            gyro_measurements should be updated
            for now, let's use delay function
        */

        static uint64_t last_loop_time;
        uint16_t loop_frequency = 1000; // [Hz]
        float motor_max = 50.f;         // % of max motor speed you want use 0 = 0% - no spin; 100 = 100% - max speed
        int8_t add;

        if (get_global_time() - last_loop_time > 1000000 / loop_frequency)
        {
            last_loop_time = get_global_time();

            // update value of each motor variable (value has to be in range 2000-4000):
            if (motor_1_value >= 4000 - motor_max / 100 * 2000)
            {
                add = -1;
            }
            else if (motor_1_value <= 2000)
            {
                add = 1;
                // send 0 (1953) to ESC (it will activate ESC):
                uint64_t time_stamp = get_global_time();
                motor_1_value = 1953;
                motor_2_value = 1953;
                motor_3_value = 1953;
                motor_4_value = 1953;

                // wait for some time with no spining motors
                // you can change it but for the first activation you need to send 1953 for >= 0.5 [s])
                while (get_global_time() - time_stamp < 3 * 1000000)
                {
                    update_motors();
                    delay_micro(1000);
                }

                // after esc activation you can send value in range 2000-4000:
                motor_1_value = 2000;
                motor_2_value = 2000;
                motor_3_value = 2000;
                motor_4_value = 2000;
            }

            // accelerate or decelerate motors:
            motor_1_value += add;
            motor_2_value += add;
            motor_3_value += add;
            motor_4_value += add;

            // send BDshot frame and receive ESC response (updating motors rpm values is inside this function):
            update_motors();

            // update coefficients of notches for new rpms:
            RPM_filter_update(&rpm_filter_gyro);

            // next apply RPM filtering (for each measurements and axes):
            gyro_measurements[0] = RPM_filter_apply(&rpm_filter_gyro, 0, gyro_measurements[0]);
            gyro_measurements[1] = RPM_filter_apply(&rpm_filter_gyro, 1, gyro_measurements[1]);
            gyro_measurements[2] = RPM_filter_apply(&rpm_filter_gyro, 2, gyro_measurements[2]);
        }
        else
        {
            // sleep for a bit:
            delay_micro(10);
        }
    }
    return 0;
}

//  functions only for above sample code. It is not essential for your code:

void delay_micro(uint16_t delay_time)
{
    uint64_t t = get_global_time();
    while (get_global_time() - t < delay_time)
    {
        ;
    }
}

uint64_t get_global_time()
{
    // gives time in [us] with accuracy of 1 [us]
    return (global_time + TIM5->CNT);
}

void TIM5_IRQHandler()
{
    if (TIM_SR_UIF & TIM5->SR)
    {
        TIM5->SR &= ~TIM_SR_UIF;
        global_time += (TIM5->ARR + 1);
    }
}

static void setup_TIM5(void)
{
    // enable TIM5 clock:
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    // Timer clock is 84 [MHz]
    // 32bit cnt register
    // register is buffered and only overflow generate interrupt:
    TIM5->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

    TIM5->PSC = 84 - 1;       // every 1 us 1 count
    TIM5->ARR = 10000000 - 1; // 1 period is 10 [s] long

    //	interrupt enable:
    TIM5->DIER |= TIM_DIER_UIE;

    //	TIM5 enabling:
    TIM5->EGR |= TIM_EGR_UG;
    TIM5->CR1 |= TIM_CR1_CEN;

    //	nvic interrupt enable (TIM5 interrupt);
    NVIC_EnableIRQ(TIM5_IRQn);
    NVIC_SetPriority(TIM5_IRQn, 7);
}