#include "I2C1_ADC1.h"
#include "Clock_Syst.h"

I2C_Status I2C_Stat;

void I2C1_Init(void) {
	RCC->APB1ENR |= (1U << 21);  // enable I2C1 CLOCK

	GPIOB->MODER |= (2U << 12) | (2U << 14); // PB6 & PB7 Alternate function
	GPIOB->OTYPER |= (1U << 6) | (1U << 7); //  open drain
	GPIOB->OSPEEDR |= (3U << 12) | (3U << 14); // High speed
	GPIOB->PUPDR |= (1U << 12) | (1U << 14); // Enable pull ups
	GPIOB->AFR[0] |= (4U << 24) | (4U << 28); // Alternate function = I2C1

	I2C1->CR1 |= (1U << 15);  // reset the I2C1
	I2C1->CR1 &= ~(1U << 15);  // Normal operation

	I2C1->CR2 |= (4U << 0);  // PCLK1 FREQUENCY in MHz
	I2C1->CCR = 20U;  // check the datasheet P.106 to find the values of tr(SCL)= 1000ns & tw(SCLH)= 4us in Sm mode
	I2C1->TRISE = 5U;

	I2C1->CR1 |= (1U << 0);  // Enable I2C1
}
void I2C1_Write(uint8_t Address, uint8_t *Data, uint8_t size, uint8_t Timeout) {
	uint32_t prev = 0;
	I2C1->CR1 |= (1U << 8) | (1U << 10);  // Generate START
	prev = GetTick();
	while (!(I2C1->SR1 & I2C_SR1_SB)) {
		if ((GetTick() - prev) > Timeout) {
			I2C1->CR1 |= (1U << 9);
			I2C_Stat = START_KO;
			return;
		}
	}
	I2C1->DR = (Address << 1) | 0U;  //  send the address in writing mode | 0
	prev = GetTick();
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) { // Good Address ?
		if ((GetTick() - prev) > Timeout) {
			I2C1->CR1 |= (1U << 9);
			I2C_Stat = ADDR_KO_TX;
			return;
		}
	}
	uint16_t temp = I2C1->SR1 | I2C1->SR2; // read SR1 and SR2 to clear the ADDR bit

	prev = GetTick();
	for (uint8_t i = 0; i < size; i++) {
		while (!(I2C1->SR1 & I2C_SR1_TXE)) { // DR Empty?
			if ((GetTick() - prev) > Timeout) {
				I2C1->CR1 |= (1U << 9);
				I2C_Stat = KO_TX;
				return;
			}
		}

		I2C1->DR = Data[i]; // Send data

		prev = GetTick();
		while (!(I2C1->SR1 & I2C_SR1_BTF)) { // Transmission completed ?
			if ((GetTick() - prev) > Timeout) {
				I2C1->CR1 |= (1U << 9);
				I2C_Stat = KO_TX;
				return;
			}
		}

	}
	I2C1->CR1 |= (1U << 9); // STOP I2C
}

void I2C1_Read(uint8_t Address, uint8_t *DataW, uint8_t sizeW, uint8_t *DataR,
		uint8_t sizeR, uint8_t Timeout) {
	uint32_t prev = 0;

	I2C1_Write(Address, DataW, sizeW, Timeout);

	I2C1->CR1 |= (1U << 8) | (1U << 10);  // Generate START
	// Enable the ACK
	prev = GetTick();
	while (!(I2C1->SR1 & I2C_SR1_SB)) {
		if ((GetTick() - prev) > Timeout) {
			I2C1->CR1 |= (1U << 9);
			I2C_Stat = START_KO;
			return;
		}
	}
	I2C1->DR = (Address << 1) | 1U;  //  send the address in reading mode | 1U
	prev = GetTick();
	while (!(I2C1->SR1 & I2C_SR1_ADDR)) { // Good Address?
		if ((GetTick() - prev) > Timeout) {
			I2C1->CR1 |= (1U << 9);
			I2C_Stat = ADDR_KO_RX;
			return;
		}
	}

	uint16_t temp = I2C1->SR1 | I2C1->SR2; // read SR1 and SR2 to clear the ADDR bit

	for (uint8_t i = 0; i < sizeR; i++) {
		if (i + 1 == sizeR) {
			I2C1->CR1 &= ~I2C_CR1_ACK;  // The STM32 sends a NACK once it gets the final byte
			I2C1->CR1 |= (1U << 9);   // I2C STOP
		}
		prev = GetTick();
		while (!(I2C1->SR1 & I2C_SR1_RXNE)) { // DR Not Empty?
			if ((GetTick() - prev) > Timeout) {
				I2C1->CR1 |= (1U << 9);
				I2C_Stat = KO_RX;
				return;
			}
		}

		DataR[i] = I2C1->DR; // Store the recieved data
	}
	I2C1->CR1 |= (1U << 9); // I2C STOP not mandatory since it was already stopped above
	I2C_Stat = OK_RX;
}
void ADC1_Init(void){

	GPIOA->MODER |= (3U<<0)|(3U<<2);  // analog mode for PA0 & PA1

	RCC->APB2ENR |= (1U<<8);  // enable ADC1 clock

//	ADC1->CR2 &= ~(1U<<1); // Continuous conversion disabled
	ADC1->CR2 |= (1U<<8);// Enable DMA for ADC
	ADC1->CR2 |= (1U<<9);// Enable Continuous DMA Request

	ADC1->SMPR2 |= (1U<<3) | (1U<<0);  // Sampling time of 15 cycles for channel 0 and channel 1
	ADC1->SQR1 |= (1U<<20);   // SQR1_L =1 for 2 conversions
	ADC1->SQR3 |= (0U<<0)|(1U<<5);  // SEQ1 for Channel 0 SEQ2 for CHannel 1
	ADC1->CR1 |= (1U<<8);    // SCAN mode enable
	ADC1->CR2 |= (1U<<0);   // ADON =1 enable ADC1
	uint32_t delay = 10000;
	while (delay--);// Wait for ADC to stabilize
	ADC1->CR2 |= ADC_CR2_SWSTART;  // start the conversion
}