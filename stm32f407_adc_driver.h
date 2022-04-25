/*
 * stm32f407_adc_driver.h
 *
 *  Created on: 1 Mar 2022
 *      Author: USER
 */
#include <stm32f407xx.h>

#ifndef INC_STM32F407_ADC_DRIVER_H_
#define INC_STM32F407_ADC_DRIVER_H_

typedef struct {
	uint8_t ADC_Type;					//Possible value from @ADC_Type
	uint8_t ADC_Resoltuion;				//Possible value from @ADC_Resolution
	uint8_t ADC_Mode;					//Possible value from @ADC_Mode
	uint8_t ADC_Alligned;				//Possible value from @ADC_Alligned
}ADC_Config_t;


typedef struct{
	ADC_RegDef_t *pADCx; 				//This holds the base address of the GPIO port to which the pin belongs. Pointer GPIOx will point to address of GPIO mentioned.
	ADC_Config_t ADC_PinConfig;		//This holds GPIO pin configuration settings
}ADC_Handle_t;


//ADC Type

#define ADC_Type_ADC1
#define ADC_Type_ADC2
#define ADC_Type_ADC3








#endif /* INC_STM32F407_ADC_DRIVER_H_ */
