#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "bdshot.h"

static void fill_bb_BDshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value, uint16_t m4_value);
static void update_motors_rpm();
static uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift);
static void read_BDshot_response(uint32_t value, uint8_t motor);
static bool BDshot_check_checksum(uint16_t value);
static uint16_t prepare_BDshot_package(uint16_t value);
static uint16_t calculate_BDshot_checksum(uint16_t value);

// flags for reception or transmission:
static bool bdshot_reception_1 = true;
static bool bdshot_reception_2 = true;

void DMA2_Stream6_IRQHandler(void)
{

    if (DMA2->HISR & DMA_HISR_TCIF6)
    {
        DMA2->HIFCR |= DMA_HIFCR_CTCIF6;

        if (bdshot_reception_1)
        {
            // set GPIOs as inputs:
            GPIOA->MODER &= ~GPIO_MODER_MODER2;
            GPIOA->MODER &= ~GPIO_MODER_MODER3;
            // set pull up for those pins:
            GPIOA->PUPDR |= GPIO_PUPDR_PUPDR2_0 | GPIO_PUPDR_PUPDR3_0;

            // set timer:
            TIM1->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING - 1;
            TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING;

            DMA2_Stream6->CR &= ~(DMA_SxCR_DIR);
            DMA2_Stream6->PAR = (uint32_t)(&(GPIOA->IDR));
            DMA2_Stream6->M0AR = (uint32_t)(dshot_bb_buffer_1_4_r);
            // Main idea:
            // After sending DShot frame to ESC start receiving GPIO values.
            // Capture data (probing longer than ESC response).
            // There is ~33 [us] gap before the response so it is necessary to add more samples:
            DMA2_Stream6->NDTR = ((int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);

            DMA2_Stream6->CR |= DMA_SxCR_EN;
            bdshot_reception_1 = false;
        }
    }

    if (DMA2->HISR & DMA_HISR_HTIF6)
    {
        DMA2->HIFCR |= DMA_HIFCR_CHTIF6;
    }
    if (DMA2->HISR & DMA_HISR_DMEIF6)
    {
        DMA2->HIFCR |= DMA_HIFCR_CDMEIF6;
    }
    if (DMA2->HISR & DMA_HISR_TEIF6)
    {
        DMA2->HIFCR |= DMA_HIFCR_CTEIF6;
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    if (DMA2->LISR & DMA_LISR_TCIF2)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;

        if (bdshot_reception_2)
        {
            // set GPIOs as inputs:
            GPIOB->MODER &= ~GPIO_MODER_MODER0;
            GPIOB->MODER &= ~GPIO_MODER_MODER1;
            // set pull up for these pins:
            GPIOB->PUPDR |= GPIO_PUPDR_PUPDR0_0 | GPIO_PUPDR_PUPDR1_0;

            // set timer:
            TIM8->ARR = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING - 1;
            TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH * DSHOT_MODE / BDSHOT_RESPONSE_BITRATE / BDSHOT_RESPONSE_OVERSAMPLING;

            DMA2_Stream2->CR &= ~(DMA_SxCR_DIR);
            DMA2_Stream2->PAR = (uint32_t)(&(GPIOB->IDR));
            DMA2_Stream2->M0AR = (uint32_t)(dshot_bb_buffer_2_3_r);
            // Main idea:
            // After sending DShot frame to ESC start receiving GPIO values.
            // Capture data (probing longer than ESC response).
            // There is ~33 [us] gap before the response so it is necessary to add more samples:
            DMA2_Stream2->NDTR = ((int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 + BDSHOT_RESPONSE_LENGTH + 1) * BDSHOT_RESPONSE_OVERSAMPLING);

            DMA2_Stream2->CR |= DMA_SxCR_EN;
            bdshot_reception_2 = false;
        }
    }

    if (DMA2->LISR & DMA_LISR_HTIF2)
    {
        DMA2->LIFCR |= DMA_LIFCR_CHTIF2;
    }
    if (DMA2->LISR & DMA_LISR_DMEIF2)
    {
        DMA2->LIFCR |= DMA_LIFCR_CDMEIF2;
    }
    if (DMA2->LISR & DMA_LISR_TEIF2)
    {
        DMA2->LIFCR |= DMA_LIFCR_CTEIF2;
    }
}

void update_motors()
{
    // prepare for sending:
    update_motors_rpm();

    fill_bb_BDshot_buffer(prepare_BDshot_package(*motor_1_value_pointer),
                          prepare_BDshot_package(*motor_2_value_pointer),
                          prepare_BDshot_package(*motor_3_value_pointer),
                          prepare_BDshot_package(*motor_4_value_pointer));

    bdshot_reception_1 = true;
    bdshot_reception_2 = true;

    // set GPIOs as output:
    GPIOB->MODER |= GPIO_MODER_MODER0_0;
    GPIOB->MODER |= GPIO_MODER_MODER1_0;
    GPIOA->MODER |= GPIO_MODER_MODER2_0;
    GPIOA->MODER |= GPIO_MODER_MODER3_0;

    DMA2_Stream6->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream6->PAR = (uint32_t)(&(GPIOA->BSRRL));
    DMA2_Stream6->M0AR = (uint32_t)(dshot_bb_buffer_1_4);
    DMA2_Stream6->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

    DMA2_Stream2->CR |= DMA_SxCR_DIR_0;
    DMA2_Stream2->PAR = (uint32_t)(&(GPIOB->BSRRL));
    DMA2_Stream2->M0AR = (uint32_t)(dshot_bb_buffer_2_3);
    DMA2_Stream2->NDTR = DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS;

#if defined(BIT_BANGING_V1)

    // Main idea:
    // Every bit frame is divided in sections and for each section DMA request is generated.
    // After some sections (at beginning, after 0-bit time and after 1-bit time) to GPIO register can be sent value to set 1 or to set 0.
    // For rest of the sections 0x0 is sent so GPIOs don't change values.
    // It uses only 1 CCR on each timer.
    // Idea for reception is the same.

    //	TIM1 setup:
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;
    TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

    // TIM8 setup:
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

    //  send:
    DMA2_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;

    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;

#elif defined(BIT_BANGING_V2)

    // Main idea:
    // DMA requests generated at beginning, after 0-bit time and after 1-bit time
    // for each bit there is only 3 sections -> buffers are much smaller than in version 1
    // but uses 3 CCR for each timer (probably not big deal)
    // It works for transferring but for reception it is not useful

    //	TIM1 setup:
    TIM1->CCR1 = 0;
    TIM1->CCR2 = DSHOT_BB_0_LENGTH;
    TIM1->CCR3 = DSHOT_BB_1_LENGTH;
    TIM1->ARR = DSHOT_BB_FRAME_LENGTH - 1;

    // TIM8 setup:
    TIM8->CCR1 = 0;
    TIM8->CCR2 = DSHOT_BB_0_LENGTH;
    TIM8->CCR3 = DSHOT_BB_1_LENGTH;
    TIM8->ARR = DSHOT_BB_FRAME_LENGTH - 1;

    //  send:
    TIM1->EGR |= TIM_EGR_UG;
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->CR1 |= TIM_CR1_CEN;

    DMA2_Stream6->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;
#endif
}

#if defined(BIT_BANGING_V1)

void preset_bb_BDshot_buffers()
{
    // these values are constant so they should be set once in the setup routine:
    for (uint16_t i = 0; i < DSHOT_BB_BUFFER_LENGTH * DSHOT_BB_FRAME_SECTIONS; i++)
    {
        // set all bits to 0x00. after that state of GPIOs outputs will stay the same:
        dshot_bb_buffer_1_4[i] = 0x00;
        dshot_bb_buffer_2_3[i] = 0x00;
    }
    for (uint8_t i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++)
    {
        // 2 last bit will stay always high (it is for ESC to capture dshot frame to the end)
        // each bit is starting with lowering edge and after DSHOT_BB_1_LENGTH is rising (for 0-bit it rises earlier but always is high after 1-bit time)
        // set low edge at the beginning of each bit:
        dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_1 | GPIO_BSRR_BR_0 << MOTOR_4;
        dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_2 | GPIO_BSRR_BR_0 << MOTOR_3;
        // set high after 1-bit length:
        dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_1_LENGTH - 1] = GPIO_BSRR_BS_0 << MOTOR_1 | GPIO_BSRR_BS_0 << MOTOR_4;
        dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_1_LENGTH - 1] = GPIO_BSRR_BS_0 << MOTOR_2 | GPIO_BSRR_BS_0 << MOTOR_3;
    }
}

static void fill_bb_BDshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value,
                                  uint16_t m4_value)
{
    // For each bite DMA is transfering DSHOT_BB_FRAME_SECTION times data into GPIO register. But we need to decide only about 3 values (begining, 0-bit time, 1-bit time).
    // For Rest values we send 0x0 into register so GPIOs stay the same.
    // Morover each bite is preset (lowering edge at first and rising edge after DSHOT_1_length rest values are 0x0 so there will be no changes in GPIO uotput registers).
    // Now it is needed to only decide about rising edge after DSHOT_0_length (if bit is 0) or seting 0 so LOW state will stay until DSHOT_1_length.
    // In addition 2 last bites are set always high (ESC needs time for proper signal detection and those high values define end off the transmission).

    for (uint8_t i = 0; i < DSHOT_BB_BUFFER_LENGTH - 2; i++) // last 2 bits are always high (logic 0)
    {
        if (1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i) & m1_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] = 0x00;
        }
        else
        {
            // if bit is zero set rising edge after DSHOT_0_length:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] = GPIO_BSRR_BS_0 << MOTOR_1;
        }
        if (1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i) & m2_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] = 0x00;
        }
        else
        {
            // if bit is zero set rising edge after DSHOT_0_length:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] = GPIO_BSRR_BS_0 << MOTOR_2;
        }
        if (1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i) & m3_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] |= 0x00;
        }
        else
        {
            // if bit is zero set rising edge after DSHOT_0_length:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] |= GPIO_BSRR_BS_0 << MOTOR_3;
        }
        if (1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i) & m4_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] |= 0x00;
        }
        else
        {
            // if bit is zero set rising edge after DSHOT_0_length:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + DSHOT_BB_0_LENGTH - 1] |= GPIO_BSRR_BS_0 << MOTOR_4;
        }
    }
}
#elif defined(BIT_BANGING_V2)

void preset_bb_BDshot_buffers()
{

    // this values are constant so they can be set once in the setup rutine:

    // make 2 high frames after Dshot frame:
    for (uint8_t i = 0; i < DSHOT_BB_FRAME_SECTIONS * 2; i++)
    {
        dshot_bb_buffer_1_4[(DSHOT_BUFFER_LENGTH)*DSHOT_BB_FRAME_SECTIONS - i - 1] = GPIO_BSRR_BS_0 << MOTOR_1 | GPIO_BSRR_BS_0 << MOTOR_4;
        dshot_bb_buffer_2_3[(DSHOT_BUFFER_LENGTH)*DSHOT_BB_FRAME_SECTIONS - i - 1] = GPIO_BSRR_BS_0 << MOTOR_2 | GPIO_BSRR_BS_0 << MOTOR_3;
    }
    for (uint8_t i = 0; i < (DSHOT_BB_BUFFER_LENGTH - 2); i++) // last 2 bits are always high (logic 0)
    {
        //  first section always lower edge:
        dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_1 | GPIO_BSRR_BR_0 << MOTOR_4;
        dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS] = GPIO_BSRR_BR_0 << MOTOR_2 | GPIO_BSRR_BR_0 << MOTOR_3;

        // last section always rise edge:
        dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 2] = GPIO_BSRR_BS_0 << MOTOR_1 | GPIO_BSRR_BS_0 << MOTOR_4;
        dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 2] = GPIO_BSRR_BS_0 << MOTOR_2 | GPIO_BSRR_BS_0 << MOTOR_3;
    }
}

static void fill_bb_BDshot_buffer(uint16_t m1_value, uint16_t m2_value, uint16_t m3_value,
                                  uint16_t m4_value)
{
    // each bite frame is divided in sections where slope can vary:
    for (uint8_t i = 0; i < (DSHOT_BB_BUFFER_LENGTH - 2); i++) // last 2 bits are always high (logic 0)
    {
        if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m1_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 1] = 0x00;
        }
        else
        {
            // if bit is zero set high:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 1] = GPIO_BSRR_BS_0 << MOTOR_1;
        }
        if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m2_value)
        {
            // if bit is one send 0x00 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 1] = 0x00;
        }
        else
        {
            // if bit is zero set high:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 1] = GPIO_BSRR_BS_0 << MOTOR_2;
        }
        if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m3_value)
        {
            // if bit is one send 0 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 1] |= 0x00;
        }
        else
        {
            // if bit is zero set high:
            dshot_bb_buffer_2_3[i * DSHOT_BB_FRAME_SECTIONS + 1] |= GPIO_BSRR_BS_0 << MOTOR_3;
        }
        if ((1 << (DSHOT_BB_BUFFER_LENGTH - 3 - i)) & m4_value)
        {
            // if bit is one send 0 so that GPIOs output will not change (will stay low):
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 1] |= 0x00;
        }
        else
        {
            // if bit is zero set high:
            dshot_bb_buffer_1_4[i * DSHOT_BB_FRAME_SECTIONS + 1] |= GPIO_BSRR_BS_0 << MOTOR_4;
        }
    }
}
#endif

static void update_motors_rpm()
{
    // BDshot bit banging reads whole GPIO register.
    // Now it's time to create BDshot responses from all motors (made of individual bits).
    uint32_t motor_1_response = get_BDshot_response(dshot_bb_buffer_1_4_r, MOTOR_1);
    uint32_t motor_2_response = get_BDshot_response(dshot_bb_buffer_2_3_r, MOTOR_2);
    uint32_t motor_3_response = get_BDshot_response(dshot_bb_buffer_2_3_r, MOTOR_3);
    uint32_t motor_4_response = get_BDshot_response(dshot_bb_buffer_1_4_r, MOTOR_4);

    read_BDshot_response(motor_1_response, 1);
    read_BDshot_response(motor_2_response, 2);
    read_BDshot_response(motor_3_response, 3);
    read_BDshot_response(motor_4_response, 4);
}

static uint32_t get_BDshot_response(uint32_t raw_buffer[], const uint8_t motor_shift)
{
    // Reception starts just after transmission, so there is a lot of HIGH samples. Find first LOW bit:

    uint16_t i = 0;
    uint16_t previous_i = 0;
    uint16_t end_i = 0;
    uint32_t previous_value = 1;
    uint32_t motor_response = 0;
    uint8_t bits = 0;

    while (i < (int)(33 * BDSHOT_RESPONSE_BITRATE / 1000 * BDSHOT_RESPONSE_OVERSAMPLING))
    {
        if (!(raw_buffer[i] & (1 << motor_shift)))
        {
            previous_value = 0;
            previous_i = i;
            end_i = i + BDSHOT_RESPONSE_LENGTH * BDSHOT_RESPONSE_OVERSAMPLING;
            break;
        }
        i++;
    }
    // if LOW edge was detected:
    if (previous_value == 0)
    {
        while (i < end_i)
        {
            // then look for changes in bits values and compute BDSHOT bits:
            if ((raw_buffer[i] & (1 << motor_shift)) != previous_value)
            {
                const uint8_t len = MAX((i - previous_i) / BDSHOT_RESPONSE_OVERSAMPLING, 1); // how many bits had the same value
                bits += len;
                motor_response <<= len;
                if (previous_value != 0)
                {
                    motor_response |= (0x1FFFFF >> (21 - len)); // 21 ones right-shifted by 20 or less
                }
                previous_value = raw_buffer[i] & (1 << motor_shift);
                previous_i = i;
            }
            i++;
        }
        // if last bits were 1 they were not added so far
        motor_response <<= (BDSHOT_RESPONSE_LENGTH - bits);
        motor_response |= 0x1FFFFF >> bits; // 21 ones right-shifted

        return motor_response;
    }
    else
    { // if LOW edge was not found return incorrect motor response:
        return 0xFFFFFFFF;
    }
}

static void read_BDshot_response(uint32_t value, uint8_t motor)
{
    // BDshot frame contain 21 bytes but first is always 0 (used only for detection).
    // Next 20 bits are 4 sets of 5-bits which are mapped with 4-bits real value.
    // After all, value is 16-bit long with 12-bit eRPM value (actually it is a period of eRPM) and 4-bit CRC.
    // 12-bit eRPM value has 3 first bits od left shifting and 9-bit mantisa.

    // put nibbles in the array in places of mapped values (to reduce empty elements smallest mapped value will always be subtracted)
    // now it is easy to create real value - mapped value indicate array element which contain nibble value:
#define iv 0xFFFFFFFF
    static const uint32_t GCR_table[32] = {
        iv, iv, iv, iv, iv, iv, iv, iv, iv, 9, 10, 11, iv, 13, 14, 15,
        iv, iv, 2, 3, iv, 5, 6, 7, iv, 0, 8, 1, iv, 4, 12, iv};

    value = (value ^ (value >> 1)); // now we have GCR value

    uint32_t decoded_value = GCR_table[(value & 0x1F)];
    decoded_value |= GCR_table[((value >> 5) & 0x1F)] << 4;
    decoded_value |= GCR_table[((value >> 10) & 0x1F)] << 8;
    decoded_value |= GCR_table[((value >> 15) & 0x1F)] << 12;

    // if wrongly decoded decoded_value will be bigger than uint16_t:
    if (decoded_value < 0xFFFF && BDshot_check_checksum(decoded_value))
    {
        // if checksum is correct real save real RPM.
        // value sent by ESC is a period between each pole changes [us].
        // to achive eRPM we need to find out how many of these changes are in one minute.
        // eRPM = (60*1000 000)/T_us next RPM can be achived -> RPM = eRPM/(poles/2):

        motors_rpm[motor - 1] = ((decoded_value & 0x1FF0) >> 4) << (decoded_value >> 13);      // cut off CRC and add shifting - this is period in [us]
        motors_rpm[motor - 1] = 60 * 1000000 / motors_rpm[motor - 1] * 2 / MOTOR_POLES_NUMBER; // convert to RPM
        motors_error[motor - 1] = 0.9 * motors_error[motor - 1];                               // reduce motor error
    }
    else
    {
        motors_error[motor - 1] = 0.9 * motors_error[motor - 1] + 10; // increase motor error
    }
}

static bool BDshot_check_checksum(uint16_t value)
{
    // BDshot frame has 4 last bits CRC:
    if (((value ^ (value >> 4) ^ (value >> 8) ^ (value >> 12)) & 0x0F) == 0x0F)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static uint16_t prepare_BDshot_package(uint16_t value)
{
    // value is in range of 2000-4000 so I need to transform it into Dshot range (48-2047)
    value -= 1953;
    if (value > 0 && value < 48)
    {
        value = 48;
    }
    return ((value << 5) | calculate_BDshot_checksum(value));
}

static uint16_t calculate_BDshot_checksum(uint16_t value)
{
    // 12th bit for telemetry on/off (1/0):
    value = value << 1;

    return (~(value ^ (value >> 4) ^ (value >> 8))) & 0x0F;
}