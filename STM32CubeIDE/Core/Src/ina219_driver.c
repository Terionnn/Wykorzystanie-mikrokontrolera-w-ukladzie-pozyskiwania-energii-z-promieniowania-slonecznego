

#include "main.h"
#include "ina219_driver.h"


extern I2C_HandleTypeDef hi2c2;

void INA219_Reset()
{
	uint16_t data = 0x8000;
    uint8_t reg[2];
    reg[0] = (data >> 8) & 0xFF;
    reg[1] = (data >> 0) & 0xFF;
    HAL_I2C_Mem_Write(&hi2c2, INA219_ADDR, 0x00, 1, reg, 2, 100);
}

void INA219_Calibrate()
{
	uint16_t config = 0x2000 | 0x1800 | 0x0180 | 0x0018 | 0x07;
	uint16_t calibration = 9840;
	uint8_t reg[2];

	reg[0] = (calibration >> 8) & 0xFF;
	reg[1] = (calibration >> 0) & 0xFF;
    HAL_I2C_Mem_Write(&hi2c2, INA219_ADDR, 0x05, 1, reg, 2, 100);

    reg[0] = (config >> 8) & 0xFF;
    reg[1] = (config >> 0) & 0xFF;
    HAL_I2C_Mem_Write(&hi2c2, INA219_ADDR, 0x00, 1, reg, 2, 100);
}

uint16_t INA219_ReadCurrent()
{
	uint8_t result[2];
	HAL_I2C_Mem_Read(&hi2c2, INA219_ADDR, 0x04, 1, result, 2, 100);
	uint16_t shifted = ((result[0] << 8) | result[1]);
	return shifted;
}



