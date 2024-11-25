#include "Clock_Syst.h"

volatile uint32_t TimeDelay;
volatile uint32_t Ticks;

void ClockInit(void) {
		
	RCC->APB1ENR |= (1U << 28); // PWR CLOCK ENABLE
	PWR->CR |= (3U << 14); // VOLTAGE REGULATOR SCALE 1

	RCC->CR |= (1U << 0); // HSI ON
	while (!(RCC->CR & (1U << 1))) // HSI ON ?
		;
// Flash Configuration - Check RCC in .ioc file for reference
	FLASH->ACR |= (7U << 8);
	FLASH->ACR &= ~(0xFU << 0); // Latency 0WS

// Prescalars
	RCC->CFGR |= (10U << 4); // AHB presc
	RCC->CFGR |= (4U << 10); //APB1 presc
	RCC->CFGR &= ~(1U << 15); //APB2 presc

// PLLs

	RCC->PLLCFGR |= (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | (PLL_Q << 24);
	RCC->PLLCFGR &= ~(1U << 22); // HSI as PLL source

// PLL
	RCC->CR |= (1U << 24); // PLL ON
	while (!(RCC->CR & (1U << 25)))
		;

// Clock source
	RCC->CFGR |= (2U << 0); // PLL as System Clock
	while (!(RCC->CFGR & (2U << 2)))
		;
}

void SysTick_Init(void) {
	SysTick->CTRL &= ~(7U << 0); // Disable SysTick
	SysTick->LOAD = 8000 - 1; // Set reload register
	NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1); // Assign the lowest priority (highest number)
	SysTick->VAL = 0; // Reset the SysTick counter value
	SysTick->CTRL |= 7U << 0; // Enable Systick
}

void SysTick_Handler(void) { // SysTick interrupt service routine
	Ticks++;
	if (TimeDelay > 0)
		TimeDelay--;
}

uint32_t GetTick(void) {
	return Ticks;
}

void Delay(uint32_t nTime) {
	TimeDelay = nTime;
	while (TimeDelay != 0)
		;
}