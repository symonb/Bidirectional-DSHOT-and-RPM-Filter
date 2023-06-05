
#include "stm32f4xx.h"
#include "global_constants.h"
#include "global_variables.h"
#include "bdshot.h"
#include "setup.h"

/* IMPORTANT:
 APB2 max frequency is 84 [MHz], 168 [MHz] only for timers
 APB1 max frequency is 42 [MHz], 84 [MHz] only for timers
 */

static void setup_HSE();
static void setup_PLL();
static void setup_GPIOA();	// GPIOA (pin 2 - motor; pin 3 - motor)
static void setup_GPIOB();	// GPIOB (pin 0 - motor; pin 1 - motor)
static void setup_BDshot(); // Bidirectional DShot
static void setup_DMA();

void setup()
{
	// basic configuration:
	setup_HSE();
	setup_PLL();
	// BDshot specific setup:
	setup_GPIOA();
	setup_GPIOB();
	setup_BDshot();
	setup_DMA();
}

static void setup_HSE()
{
	RCC->CFGR = (RCC->CFGR & (uint32_t)(~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; /* (2) */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
	{
		; // wait
	}
	// enable HSE:
	RCC->CR |= RCC_CR_HSEON;
	while (0 == (RCC->CR & RCC_CR_HSERDY))
	{
		// waiting until RDY bit is set
	}
	// set all prescalers (main clock will be set at 168 MHz via PLL):

	RCC->CFGR |= RCC_CFGR_PPRE2_2;					  // APB2 presc. = 2
	RCC->CFGR |= RCC_CFGR_PPRE1_2 | RCC_CFGR_PPRE1_0; // APB1 presc. = 4
	// AHB presc. = 1 (left at default)

	// enable power interface clock
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
}

static void setup_PLL()
{

	/* (1) Test if PLL is used as System clock */
	/* (2) Select HSI as system clock */
	/* (3) Wait for HSI switched */
	/* (4) Disable the PLL */
	/* (5) Wait until PLLRDY is cleared */
	/* (6) Configure flash */
	/* (7) Set HSE as PLL source */
	/* (8) Set the PLLM to 4, PLLN to 168, PLLP to 2, PLLQ to 7 */
	/* PLL_freq = PLL_clock_in / PLLM * PLLN / PLLP
	 * PLL_48 =PLL_clock_in / PLLM * PLLN / PLLQ = 48 [MHz]
	 * Important:
	 * 2 <= PLLM <= 64;
	 * 50 <= PLLN <= 432;
	 * PLLP = {2,4,6,8}
	 * 2 <= PLLQ <= 15
	 * In addition:
	 * 	1  [MHz] <= PLL_clock_in / PLLM 		 	  <=  2  [MHz] but 2 [MHz] is preferred
	 * 100 [MHz] <= PLL_clock_in / PLLM * PLLN 		  <= 432 [MHz]
	 *  			PLL_clock_in / PLLM * PLLN / PLLP <= 168 [MHz]
	 * So as I want receive max. freq. 168 [MHz] PLLP has to be 2 (VCO_out 336 [MHz]), PLLN = 168 and PLLM is 4 since 8 [MHz]/PLLM = 2 [MHz]
	 * Also PLL_48 has to be 48 [MHz] so PLLQ = 7 -> 336 [MHz] / 48 [MHz] = 7
	 * */
	/* (9) Enable the PLL */
	/* (10) Wait until PLLRDY is set */
	/* (11) Select PLL as system clock */
	/* (12) Wait until the PLL is switched on */
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) /* (1) */
	{
		RCC->CFGR = (RCC->CFGR & (uint32_t)(~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI; /* (2) */
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)				  /* (3) */
		{
			/* For robust implementation, add here time-out management */
		}
	}
	RCC->CR &= (uint32_t)(~RCC_CR_PLLON);  /* (4) */
	while ((RCC->CR & RCC_CR_PLLRDY) != 0) /* (5) */
	{
		/* For robust implementation, add here time-out management */
	}

	FLASH->ACR |= FLASH_ACR_LATENCY_5WS; /* (6) */

	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; /* (7) */

	RCC->PLLCFGR = (RCC->PLLCFGR & (~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ))) | (RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_0 | RCC_PLLCFGR_PLLQ_1 | RCC_PLLCFGR_PLLQ_2); /* (8) */

	RCC->CR |= RCC_CR_PLLON;			   /* (9) */
	while ((RCC->CR & RCC_CR_PLLRDY) == 0) /* (10) */
	{
		/* For robust implementation, add here time-out management */
	}
	RCC->CFGR |= (uint32_t)(RCC_CFGR_SW_PLL);			   /* (11) */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) /* (12) */
	{
		/* For robust implementation, add here time-out management */
	}
}

static void setup_GPIOA()
{
	// enable GPIOA clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

	//	set mode (00-input; 01-output; 10-alternate):
	// will be set in bdshot routine

	// set speed (max speed):
	GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR2 |
					   GPIO_OSPEEDER_OSPEEDR3);
}

static void setup_GPIOB()
{
	// enable GPIOB clock:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	//	set mode (00-input; 01-output; 10-alternate):
	// will be set in bdshot routine

	// set speed: (max speed)
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0 |
					   GPIO_OSPEEDER_OSPEEDR1);
}

static void setup_BDshot()
{
	//	TIM1 - only for generating time basement all outputs are set by GPIOs:

	// enable TIM1 clock:

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	// register is buffered and overflow DMA request:
	TIM1->CR1 = 0x0;
	TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// DMA request:
#if defined(BIT_BANGING_V1)
	TIM1->DIER |= TIM_DIER_CC1DE; // channel 1 request

	TIM1->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

	//	TIM1 is 168 [MHz]:
	TIM1->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM1->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

#elif defined(BIT_BANGING_V2)
	TIM1->DIER |= TIM_DIER_CC1DE; // channel 1 request
	TIM1->DIER |= TIM_DIER_CC2DE; // channel 2 request
	TIM1->DIER |= TIM_DIER_CC3DE; // channel 3 request

	TIM1->CCR1 = 0;
	TIM1->CCR2 = DSHOT_BB_0_LENGTH;
	TIM1->CCR3 = DSHOT_BB_1_LENGTH;

	//	TIM1 is 168 [MHz]:
	TIM1->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM1->ARR = DSHOT_BB_FRAME_LENGTH - 1;
#endif

	//	TIM1 enable:
	TIM1->EGR |= TIM_EGR_UG;
	TIM1->CR1 |= TIM_CR1_CEN;

	//	TIM8 - only for generating time basement all outputs are set by GPIOs:

	// enable TIM8 clock:

	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

	// register is buffered and overflow DMA request:
	TIM8->CR1 = 0x0;
	TIM8->CR1 |= TIM_CR1_ARPE | TIM_CR1_URS;

	// DMA request:
#if defined(BIT_BANGING_V1)
	TIM8->DIER |= TIM_DIER_CC1DE; // channel 1 request

	TIM8->CCR1 = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS;

	//	TIM8 is 168 [MHz]:
	TIM8->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM8->ARR = DSHOT_BB_FRAME_LENGTH / DSHOT_BB_FRAME_SECTIONS - 1;

#elif defined(BIT_BANGING_V2)
	TIM8->DIER |= TIM_DIER_CC1DE; // channel 1 request
	TIM8->DIER |= TIM_DIER_CC2DE; // channel 2 request
	TIM8->DIER |= TIM_DIER_CC3DE; // channel 3 request

	TIM8->CCR1 = 0;
	TIM8->CCR2 = DSHOT_BB_0_LENGTH;
	TIM8->CCR3 = DSHOT_BB_1_LENGTH;

	//	TIM8 is 168 [MHz]:
	TIM8->PSC = 168000 / DSHOT_MODE / DSHOT_BB_FRAME_LENGTH - 1;
	TIM8->ARR = DSHOT_BB_FRAME_LENGTH - 1;
#endif
	//	TIM8 enable:
	TIM8->EGR |= TIM_EGR_UG;
	TIM8->CR1 |= TIM_CR1_CEN;
}

static void setup_DMA()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// bidirectional DSHOT:
	// for TIM1
	DMA2_Stream6->CR = 0x0;
	while (DMA2_Stream6->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA2_Stream6->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	// all the other parameters will be set afterward

	DMA2_Stream2->CR = 0x0;
	while (DMA2_Stream2->CR & DMA_SxCR_EN)
	{
		; // wait
	}
	DMA2_Stream2->CR |= DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE | DMA_SxCR_PL_0;
	// all the other parameters will be set afterward
}

void setup_NVIC()
{
	//	nvic DMA interrupts enable:
	NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	NVIC_SetPriority(DMA2_Stream6_IRQn, 13);
	NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	NVIC_SetPriority(DMA2_Stream2_IRQn, 14);
}
