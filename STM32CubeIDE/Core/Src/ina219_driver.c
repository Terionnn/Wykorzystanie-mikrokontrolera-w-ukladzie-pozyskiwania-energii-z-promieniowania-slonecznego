

#include "main.h"
#include "ina219_driver.h"


extern I2C_HandleTypeDef hi2c2;

void INA219_Reset()
{
	uint16_t Tdata = 0x8000;
    uint8_t addr[2];
    addr[0] = (Tdata >> 8) & 0xff;
    addr[1] = (Tdata >> 0) & 0xff;
    HAL_I2C_Mem_Write(&hi2c2, INA219_ADDR, 0x00, 1, addr, 2, 1000);
}

void INA219_Calibrate()
{
	uint16_t config = 0x2000 | 0x1800 | 0x0180 | 0x0018 | 0x07;
	uint16_t ina219_calibrationValue = 9840;

	uint8_t addr[2];
	addr[0] = (ina219_calibrationValue >> 8) & 0xff;
	addr[1] = (ina219_calibrationValue >> 0) & 0xff;
    HAL_I2C_Mem_Write(&hi2c2, INA219_ADDR, 0x05, 1, addr, 2, 1000);

    addr[0] = (config >> 8) & 0xff;
    addr[1] = (config >> 0) & 0xff;
    HAL_I2C_Mem_Write(&hi2c2, INA219_ADDR, 0x00, 1, addr, 2, 1000);
}

uint16_t INA219_ReadBusVoltage()
{

	uint8_t result[2];
	HAL_I2C_Mem_Read(&hi2c2, INA219_ADDR, 0x02, 1, result, 2, 1000);
	uint16_t shifted = ((result[0] << 8) | result[1]);

	 return ((shifted >> 3  ) * 4.00);

}

uint16_t INA219_ReadCurrent()
{
	uint8_t result[2];
	HAL_I2C_Mem_Read(&hi2c2, INA219_ADDR, 0x04, 1, result, 2, 1000);
	uint16_t shifted = ((result[0] << 8) | result[1]);
	return shifted;

}

uint16_t INA219_ReadShuntVoltage()
{

	uint8_t result[2];
	HAL_I2C_Mem_Read(&hi2c2, INA219_ADDR, 0x01, 1, result, 2, 1000);
	uint16_t shifted = ((result[0] << 8) | result[1]);

	 return shifted*0.01;

}
uint16_t INA219_ReadPower()
{

	uint8_t result[2];
	HAL_I2C_Mem_Read(&hi2c2, INA219_ADDR, 0x03, 1, result, 2, 1000);
	uint16_t shifted = ((result[0] << 8) | result[1]);

	 return shifted*1.00000;

}


