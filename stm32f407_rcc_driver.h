/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: 7 Mar 2022
 *      Author: USER
 */

#ifndef INC_STM32F407_RCC_DRIVER_H_
#define INC_STM32F407_RCC_DRIVER_H_

uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F407_RCC_DRIVER_H_ */
