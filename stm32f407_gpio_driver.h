/*
 * stm32f407_gpio_driver.h
 *
 *  Created on: Jan 13, 2022
 *      Author: USER
 */

#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include <stm32f407xx.h>


typedef struct {
	uint8_t GPIO_PinNumber;				//Possible value from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;				//Possible value from @GPIO_MODE_REG
	uint8_t GPIO_PinSpeed;				//Possible value from @GPIO_SPEED
	uint8_t GPIO_PinPuPdControl;		//Possible value from @GPIO_PU_PD
	uint8_t GPIO_PinOPtype;				//Possible value from @GPIO_OUTPUT_TYPE
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

//GPIO Handler

typedef struct{
	GPIO_RegDef_t *pGPIOx; 				//This holds the base address of the GPIO port to which the pin belongs. Pointer GPIOx will point to address of GPIO mentioned.
	GPIO_PinConfig_t GPIO_PinConfig;	//This holds GPIO pin configuration settings
}GPIO_Handle_t;


//@GPIO_PIN_NUMBERS
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15


//@GPIO_MODE_REG non-interrupt modes
#define GPIO_MODE_IN				0
#define GPIO_MODE_OUT				1
#define GPIO_MODE_ALTFN				2
#define GPIO_MODE_ANALOG			3

//@GPIO_MODE_REG interrupt modes
#define GPIO_MODE_IT_FT				4		//interrupt falling edge
#define GPIO_MODE_IT_RT				5		//interrupt raising edge
#define GPIO_MODE_IT_RFT			6		//interrupt raising falling edge

//@GPIO_OUTPUT_TYPE
#define GPIO_OUT_TYPE_PP			0		//Push-pull
#define GPIO_OUT_TYPE_OD			1		//Output Drain

//@GPIO_PinSpeed
#define GPIO_OUT_SPEED_LOW			0
#define GPIO_OUT_SPEED_MEDIUM		1
#define GPIO_OUT_SPEED_FAST			2
#define GPIO_OUT_SPEED_HIGH			3

//@GPIO_PU_PD
#define GPIO_NOPUPD					0		//No push up or push down
#define GPIO_PU						1
#define GPIO_PD						2
/*************************************************************************************************************

Create API Prototypes

**************************************************************************************************************/
//Peripheral Clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Init & Deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Data Read & Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ Configuration & Handlers
void GPIO_IRQInterruptConfig(uint8_t IRQNumbers, uint8_t EnorDi );
void GPIO_IRQPriorityConfig(uint8_t IRQNumbers, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);
























#endif /* INC_STM32F407_GPIO_DRIVER_H_ */
