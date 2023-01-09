/*
 * ina219_driver.h
 *
 *  Created on: 17 sie 2022
 *      Author: Kamil
 *
 *      Na podstawie: https://github.com/komuch/PSM_INA219_STM32
 */

#ifndef INC_INA219_DRIVER_H_
#define INC_INA219_DRIVER_H_

#define INA219_ADDR (0x40<<1)

void INA219_Reset();
void INA219_Calibrate();
uint16_t INA219_ReadCurrent();


#endif
