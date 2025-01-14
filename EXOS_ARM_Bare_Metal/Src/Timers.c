#include "Timers.h"


void PWM_TIM1(void) {

	RCC->APB2ENR |= (1U << 0);	//timer clock

	TIM1->PSC = 16 - 1;
	TIM1->ARR = 10000 - 1;
	TIM1->CR1 &= ~(1U << 7); //Auto Reload preload disabled for the ARR, just to match my original code but you can enable it
	TIM1->CR1 &= ~(1U << 4); //DIR = 0 upcounting Mode

	//gpio alternate function
	GPIOA->MODER |= (2U << 16)|(2U << 18)|(2U << 20)|(2U << 22); // PA8...PA11
	GPIOA->AFR[1] |= (1U << 0)|(1U << 4)|(1U << 8)|(1U << 12); //AF1 for TIM1 in the AFRH/AFR[1] Register

	TIM1->CCMR1 &= ~(3U<<0); //Channel 1 Output Mode, same for the other channels which are already at 0
	TIM1->CCER &= ~(1U << 1);//Channel 1 active high POLARITY, same for the other channels which are already at 0

	//PWM MODE 1 for the 4 channels
	TIM1->CCMR1 |= (3U << 5)|(3U << 13);
	TIM1->CCMR2 |= (3U << 5)|(3U << 13);

	//PRELOAD Output Compare Registers for the 4 channels
	TIM1->CCMR1 |= (1U << 3)|(1U << 11); //PRELOAD OC1PE and OC2PE  => practically it means preload CCR1 and CRR2
	TIM1->CCMR2 |= (1U << 3)|(1U << 11);

	TIM1->CCER |= (1U << 0)|(1U << 4)|(1U << 8)|(1U << 12);// Enable the 4 output compare
	TIM1->BDTR |= (1U << 15);   // MOE !!! FOR TIMER 1 ONLY


	TIM1->EGR |= (1U<< 0); // Update Timer
	TIM1->CR1 |= (1U << 0);// Enable Timer
}
void PWM_TIM3(void) {

	//Output Mode,polarity and ARPE already in reset state, so to lighten the code I don't clear the bits again

	RCC->APB1ENR |= (1U << 1);	//timer clock

	TIM3->PSC = 4 - 1;
	TIM3->ARR = 20000;
	TIM3->CR1 |= (1U << 5); //updown counting mode 1 / center aligned mode 1

	//gpio alternate function
	GPIOA->MODER |= (2U << 12)|(2U << 14); // PA6 & PA7
	GPIOA->AFR[0] |= (2U << 24)|(2U << 28);//AF2 for TIM1 in the AFRL/AFR[0] Register

	TIM3->CCMR1 |= (3U << 5)|(3U << 13); //PWM MODE1
	TIM3->CCMR1 |= (1U << 3)|(1U << 11); //PRELOAD OC1PE & OC2PE
	TIM3->CCER |= (1U << 0)|(4U << 0); // Enable output compare

    TIM3->EGR |= (1U << 0);
	TIM3->CR1 |= (1U << 0);
}

void TIM9_IT(void) {
    // Enable the clock for Timer 9
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    // Timer 9 configuration
    TIM9->PSC = 1000 - 1;
    TIM9->ARR = 800 - 1;
    TIM9->DIER |= TIM_DIER_UIE; // Enable interrupt request for TIM9
    TIM9->EGR |= (1U<< 0);
    TIM9->CR1 |= TIM_CR1_CEN;// Start the timer

    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn); // Unmask the interrupt - Enable Timer 9 interrupt in NVIC
}
