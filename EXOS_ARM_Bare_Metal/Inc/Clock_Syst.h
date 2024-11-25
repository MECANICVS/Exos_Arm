#ifndef CLOCK_SYST_H
#define	CLOCK_SYST_H

#include "stm32f411xe.h"
#include <stdint.h>
// It's easier and safer to use the clock tree of Cubemx to get the PLL_x values rather than calculating them
#define PLL_M 8U
#define PLL_N 64U
#define PLL_P 0U
#define PLL_Q 4U

extern volatile uint32_t TimeDelay;
extern volatile uint32_t Ticks;

void ClockInit(void);
void SysTick_Init(void);
void SysTick_Handler(void);
uint32_t GetTick(void);
void Delay(uint32_t nTime);

#endif