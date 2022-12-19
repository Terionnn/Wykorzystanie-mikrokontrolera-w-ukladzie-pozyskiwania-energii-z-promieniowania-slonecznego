/*
 * bh1750_driver.c
 *
 *  Created on: 17 sie 2022
 *      Author: Kamil
 */

#include "main.h"
#include "bh1750_driver.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t	BH1750_PowerOn()
{
	uint8_t data = 0x01;
	return HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &data, 1, 100);
}

void BH1750_SetContinuousMode()
{
	uint8_t data = 0x10;
	HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &data, 1, 100);
	uint8_t tmp[2];
	tmp[0] = (0x40 | (31 >> 5));
	tmp[1] = (0x60 | (31 & 0x1F));
	HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &tmp[0], 1, 10);
	HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR, &tmp[1], 1, 10);
}

uint32_t BH1750_GetIlluminance()
{
	uint8_t data[2];
	uint32_t luxValue = 0;
    HAL_I2C_Master_Receive(&hi2c1, BH1750_ADDR, data, 2, 100);
    luxValue = data[0];
    luxValue <<= 8;
    luxValue += data[1];
    luxValue *= (float)69/(float)31;
    luxValue = (int)(luxValue) / 1.2;

    return luxValue;
}
