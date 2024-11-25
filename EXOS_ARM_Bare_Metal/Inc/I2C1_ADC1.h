#ifndef I2C_ADC_H
#define	I2C_ADC_H

#include "stm32f411xe.h"
#include <stdint.h>

typedef enum {
	Init, KO_TX, KO_RX, ADDR_KO_TX, ADDR_KO_RX, OK_RX, START_KO
} I2C_Status;

extern I2C_Status I2C_Stat;

void I2C1_Init(void);
void I2C1_Write(uint8_t Address, uint8_t *Data, uint8_t size, uint8_t Timeout);
void I2C1_Read(uint8_t Address, uint8_t *DataW, uint8_t sizeW, uint8_t *DataR, uint8_t sizeR, uint8_t Timeout);
void ADC1_Init(void);

#endif 