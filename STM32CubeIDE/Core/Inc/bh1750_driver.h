/*
 * bh1750_driver.h
 *
 *  Created on: 17 sie 2022
 *      Author: Kamil
 */

#ifndef INC_BH1750_DRIVER_H_
#define INC_BH1750_DRIVER_H_
#define BH1750_ADDR			(0x23<<1)

uint8_t	BH1750_PowerOn();
void BH1750_SetContinuousMode();
uint32_t BH1750_GetIlluminance();

#endif /* INC_BH1750_DRIVER_H_ */
