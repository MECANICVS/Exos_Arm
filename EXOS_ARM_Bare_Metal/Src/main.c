/**
 ******************************************************************************
 * @file           : main.c
 * @author         : B.MECANICVS
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 *
 ******************************************************************************
 */
#include "Clock_Syst.h"
#include "I2C1_ADC1.h"
#include "Timers.h"

const uint8_t PCA9685_Address = 0x40;
const uint8_t PCA9685_MODE1 = 0x00;
const uint8_t PCA9685_PRESCALE = 0xFE;
const uint8_t MODE1_SLEEP = 0x10;
const uint8_t MODE1_RESTART = 0x80;
const uint8_t MODE1_AI = 0x20;
const uint8_t PIN_0 = 0x06;
const uint32_t CLOCK = 25000000;
uint16_t Desired_frequency = 50;

/* counters and flag*/
volatile uint16_t Ovf_Cnt = 0;
uint16_t Cnt_led = 0; // for the LEDs
uint8_t Flag_mec = 0; // To trigger Arming()

/* servos' angular position */
uint8_t M1 = 0; //Big servo
uint8_t M2 = 70; //Lower Left servo
uint8_t M3 = 150; //Upper Left servo
uint8_t M4 = 0; //Lower Right servo
uint8_t M5 = 0; //Upper Right servo
uint8_t M6 = 0; //M6 = 91   when using fs90r(continous rotation servo)

/* Analog values */
uint16_t volt[2];
volatile uint16_t PR = 0; // Photo resistance value for hand detection
volatile uint16_t Bat_level = 0; // EMG Battery Level

/* Mode selection */
typedef enum {
	BASE, I2C
} MODES;

////functions definition ///////////////
static void Dragon_crest(void);
static void prescaler_choice(uint8_t address, uint16_t frequency);
static void pwm(uint8_t address, uint8_t PIN, uint16_t angle);
static uint16_t RegU(uint8_t angle);
static uint16_t RegUD(uint8_t angle);
static void Desarming(void);
static void Arming(void);
static void Rearming(void);
static void Desarming_i2c(void);
static void Arming_i2c(void);
static void Rearming_i2c(void);
static void DMA2_Init(void);
static void GPIO_Init(void);


int main(void) {
	ClockInit();
	SysTick_Init();
	GPIO_Init();
	uint32_t oldtime = GetTick();
	PWM_TIM1();
	PWM_TIM3();
	ADC1_Init();
	DMA2_Init();
	TIM9_IT();
	I2C1_Init();
	uint32_t waitTime = 0;
	uint32_t waitTime2 = 0;
	uint32_t waitTime3 = 0;
	uint32_t elapsedtime = 0;
	uint32_t prev = 0;
	MODES Mode = BASE;
	I2C_Stat = Init;
	prescaler_choice(PCA9685_Address, Desired_frequency);
	if(I2C_Stat == OK_RX){
		Mode = I2C;
		Desarming_i2c();
	} else
		Desarming();
	/* Loop forever */
	for (;;) {
		elapsedtime = GetTick() - oldtime;
		oldtime += elapsedtime;
		waitTime += elapsedtime;
		waitTime2 += elapsedtime;
		waitTime3 += elapsedtime;
		static uint8_t Flag_rearm = 0;

		Dragon_crest();

		switch (Mode) {
		case BASE:
			if ((waitTime >= 6000) && (PR < 50) && (M1 > 0)) { // Hand detected -> closing
				M1 = 0;

				TIM3->CCR1 = RegUD(M1);
				waitTime = 0;
			}

			if ((waitTime <= 3000) && (M1 == 0) && (Flag_mec == 1)) { // Muscle contracted -> arming
				Arming();

			}

			if ((waitTime > 3000) && (Flag_mec == 0) && (M1 == 0) && (PR < 50)
					&& (Flag_rearm == 0)) { // No contraction -> Reopening
				M1 = 150;

				TIM3->CCR1 = RegUD(M1);
			}

			if ((waitTime2 >= 20000) && (PR >= 80) && (M1 >= 150)) { // No Hand -> Rearming
				Rearming();
				Flag_rearm = 1;
				waitTime2 = 0;
			}

			break;

		case I2C:
			if ((waitTime >= 6000) && (PR < 50) && (M1 > 0)) {
				M1 = 0;
				pwm(PCA9685_Address, 8, M1);
				waitTime = 0;
			}

			if ((waitTime <= 3000) && (M1 == 0) && (Flag_mec == 1)) {
				Arming_i2c();
			}

			if ((waitTime > 3000) && (Flag_mec == 0) && (M1 == 0) && (PR < 50)
					&& (Flag_rearm == 0)) {
				M1 = 150;
				pwm(PCA9685_Address, 8, M1);

			}

			if ((waitTime2 >= 20000) && (PR >= 80) && (M1 >= 150)) {
				Rearming_i2c();
				Flag_rearm = 1;
				waitTime2 = 0;
			}

			break;
		}

		if (!(GPIOB->IDR & (1U << 1))) {
			NVIC_SystemReset();
		}
		if (Bat_level < 2000) {
			if (waitTime3 - prev >= 1000) {
				prev = waitTime3;
				//Toggle Red LED
				if(GPIOC->ODR & (1U << 10)){
					GPIOC->BSRR |= (1U << 26);
				}
				else{
					GPIOC->BSRR |= (1U << 10);
				}
			}
		}
	}
}



static void GPIO_Init(void){
	RCC->AHB1ENR |= (1U << 0)|(1U << 1)|(1U << 2);	//GPIOA,B & C CLOCK Enable
	GPIOB->PUPDR |= (1U << 2); //PB1 Input Pull up
	GPIOC->MODER |= (1U << 20)|(1U << 22)|(1U << 24); // PC10,11,12 Output
}

static void DMA2_Init(void){

    // Enable DMA2 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    // Select Circular mode
    DMA2_Stream0->CR |= (1U<<8);  // CIRC = 1
    // Enable Memory Address Increment
    DMA2_Stream0->CR |= (1U<<10);  // MINC = 1;
    // Set the size for data
    DMA2_Stream0->CR |= (1U<<11)|(1U<<13);  // PSIZE = 01, MSIZE = 01, 16 bit data

    DMA2_Stream0->CR |= DMA_SxCR_TCIE; // Enable Transfer Complete Interrupt
    DMA2_Stream0->CR |= DMA_SxCR_PL_0; // Priority level LOW

    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR; // Peripheral address
    DMA2_Stream0->M0AR = (uint32_t)volt; // Memory address
    DMA2_Stream0->NDTR = sizeof(volt)/sizeof(volt[0]); // Number of data items to transfer

    DMA2_Stream0->CR |= DMA_SxCR_EN; // Enable DMA stream
    NVIC_EnableIRQ(DMA2_Stream0_IRQn); // Unmask the interrupt - Enable DMA interrupt in NVIC
}

// I highly recommend to use external pull up resistors besides internal pull up for SDA & SCL
static void prescaler_choice(uint8_t address, uint16_t frequency) {
	uint8_t buffer[2];
	buffer[0] = PCA9685_MODE1;
	uint8_t initPrescaler[2];
	uint8_t oldmode;

	I2C1_Read(address, buffer, 2, &oldmode, 1, 10);
//	I2C1_Read(address, buffer, 1, &oldmode, 1, 10); // You can try this as well, sometimes it didn't work for me
	if (I2C_Stat == OK_RX) {

		uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;

		buffer[1] = newmode;

		I2C1_Write(address, buffer, 2, 10);

		uint8_t prescaler = (CLOCK / (frequency * 4096)) - 1;
		initPrescaler[0] = PCA9685_PRESCALE;
		initPrescaler[1] = prescaler;

		I2C1_Write(address, initPrescaler, 2, 10);

		buffer[1] = oldmode;

		I2C1_Write(address, buffer, 2, 10);
		Delay(5);

		buffer[1] = (oldmode | MODE1_RESTART | MODE1_AI);

		I2C1_Write(address, buffer, 2, 10);
	}
}

static void pwm(uint8_t address, uint8_t PIN, uint16_t angle) {
	uint16_t off = ((((angle * 0.0055) + 1) / 20) * 4096) - 1;
	uint8_t outputBuffer[5] = { PIN_0 + 4 * PIN, 0, (0 >> 8), off, (off >> 8) };
	I2C1_Write(address, outputBuffer, 5, 10);
}

static uint16_t RegU(uint8_t Angle) {
	return ((((Angle * 0.0055) + 1) / 20) * 10000) - 1;
}
static uint16_t RegUD(uint8_t Angle) {
	return (((Angle * 0.0055) + 1) / 20) * 20000;
}
static void Dragon_crest(void) {

	if (GPIOB->IDR & (1U << 2)) { // If muscle contracted
		Flag_mec = 1;
		Cnt_led++;
		GPIOC->BSRR |= (1U << 12); // Turn on Green LED
	} else {
		GPIOC->BSRR |= (1U << 28);
	}
	if (Ovf_Cnt > 5) {
		if (Cnt_led >= 20) {
			GPIOC->BSRR |= (1U << 11); // Turn on Blue LED for 500ms
			Ovf_Cnt = 0;
			Cnt_led = 0;
		} else {
			GPIOC->BSRR |= (1U << 27);
			Ovf_Cnt = 0;
			Cnt_led = 0;
		}
	}
}

static void Desarming(void) {
/////uncomment when using fs90r/////
/*	M6 = 75;
	TIM3->CCR2 = RegUD(M6);
	HAL_Delay(500);
	M6 = 91;     */
	TIM3->CCR2 = RegUD(M6);

	while ((M3 > 0) || (M5 < 150)) {
		TIM1->CCR2 = RegU(M3);
		TIM1->CCR4 = RegU(M5);
		M3 -= 10;
		M5 += 10;
		Delay(50);
	}
	while ((M2 > 0) || (M4 < 70)) {
		TIM1->CCR1 = RegU(M2);
		TIM1->CCR3 = RegU(M4);
		M2 -= 10;
		M4 += 10;
		Delay(50);
	}
	while (M1 < 150) {
		TIM3->CCR1 = RegUD(M1);
		M1 += 30;
		Delay(50);
	}
}

static void Arming(void) {
	/////uncomment when using fs90r/////
/*	M6 = 120;
	TIM3->CCR2 = RegUD(M6);
	HAL_Delay(500);
	M6 = 91;
	TIM3->CCR2 = RegUD(M6);  */
	M6 = 179;
	TIM3->CCR2 = RegUD(M6);

	while ((M2 < 70) || (M4 > 0)) {
		TIM1->CCR1 = RegU(M2);
		TIM1->CCR3 = RegU(M4);
		M2 += 10;
		M4 -= 10;
		Delay(50);
	}
}

static void Rearming(void) {

	while (M1 > 0) {
		TIM3->CCR1 = RegUD(M1);
		M1 -= 30;
		Delay(50);
	}
/////uncomment when using fs90r/////
/*	M6 = 120;
	TIM3->CCR2 = RegUD(M6);
	Delay(500);
	M6 = 91;
	TIM3->CCR2 = RegUD(M6);  */
	M6 = 179;
	TIM3->CCR2 = RegUD(M6);

	while ((M2 < 70) || (M4 > 0)) {
		TIM1->CCR1 = RegU(M2);
		TIM1->CCR3 = RegU(M4);
		M2 += 10;
		M4 -= 10;
		Delay(50);
	}
	while ((M3 < 150) || (M5 > 0)) {
		TIM1->CCR2 = RegU(M3);
		TIM1->CCR4 = RegU(M5);
		M3 += 10;
		M5 -= 10;
		Delay(50);
	}
}
//////////////////////////// I2C mode ///////////////////////////////////
static void Desarming_i2c(void) {

/*	M6 = 75;
	pwm(PCA9685_Address, 13, M6);
	Delay(500);
	M6 = 91;   */
	pwm(PCA9685_Address, 13, M6);

	while ((M3 > 0) || (M5 < 150)) {
		pwm(PCA9685_Address, 10, M3);
		pwm(PCA9685_Address, 12, M5);
		M3 -= 10;
		M5 += 10;
		Delay(50);
	}
	while ((M2 > 0) || (M4 < 70)) {
		pwm(PCA9685_Address, 9, M2);
		pwm(PCA9685_Address, 11, M4);
		M2 -= 10;
		M4 += 10;
		Delay(50);
	}
	while (M1 < 150) {
		pwm(PCA9685_Address, 8, M1);
		M1 += 30;
		Delay(50);
	}
}
static void Arming_i2c(void) {

/*	M6 = 120;
	pwm(PCA9685_Address, 13, M6);
	Delay(500);
	M6 = 91;
	pwm(PCA9685_Address, 13, M6);  */
	M6 = 179;
	pwm(PCA9685_Address, 13, M6);

	while ((M2 < 70) || (M4 > 0)) {
		pwm(PCA9685_Address, 9, M2);
		pwm(PCA9685_Address, 11, M4);
		M2 += 10;
		M4 -= 10;
		Delay(50);
	}
}
static void Rearming_i2c(void) {

	while (M1 > 0) {
		pwm(PCA9685_Address, 8, M1);
		M1 -= 30;
		Delay(50);
	}
/*	M6 = 120;
	pwm(PCA9685_Address, 13, M6);
	Delay(500);
	M6 = 91;
	pwm(PCA9685_Address, 13, M6); */
	M6 = 179;
	pwm(PCA9685_Address, 13, M6);

	while ((M2 < 70) || (M4 > 0)) {
		pwm(PCA9685_Address, 9, M2);
		pwm(PCA9685_Address, 11, M4);
		M2 += 10;
		M4 -= 10;
		Delay(50);
	}
	while ((M3 < 150) || (M5 > 0)) {
		pwm(PCA9685_Address, 10, M3);
		pwm(PCA9685_Address, 12, M5);
		M3 += 10;
		M5 -= 10;
		Delay(50);
	}

}

///////////////////////////////////ISR///////////////////////////////////////////////////////

void DMA2_Stream0_IRQHandler(void) {
    if (DMA2->LISR & DMA_LISR_TCIF0) { // Check transfer complete flag
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0; // note that the flag isn't cleared by doing ~DMA_LISR_TCIF0, always check the clearing mechanism
        // Handle transfer complete
    	PR = (volt[0] * 3300) / 4095;
    	Bat_level = (volt[1] * 3300) / 4095;
    }
}
void TIM1_BRK_TIM9_IRQHandler(void) {
    if (TIM9->SR & TIM_SR_UIF) { // Check if the update interrupt flag is set for Timer 9
        TIM9->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
        Ovf_Cnt++;
    	ADC1->CR2 |= ADC_CR2_SWSTART;  // restart the conversion
    }
}
