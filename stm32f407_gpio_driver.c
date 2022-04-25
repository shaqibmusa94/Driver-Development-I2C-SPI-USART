/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Jan 13, 2022
 *      Author: USER
 */

#include "stm32f407_gpio_driver.h"


/*************************************************************************************
 * @function			- GPIO_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]			- Base address of the GPIO peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == 1)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	} else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}

}

/*************************************************************************************
 * @function			- GPIO_Init
 *
 * @brief				- This function takes the argument base address (either it is the GPIO
 * 						  Port or Register Address for GPIO Mode, Speed, Output Type, etc) by dereferencing
 * 						  data type GPIO_PinConfig or pGPIOx.
 *
 * @param[in]			- Base address of GPIO Port Address or GPIO Register to Set Handlers (Mode, Speed, etc)
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	 uint32_t temp=0; //temp. register

	 //enable the peripheral clock

	 GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1 . configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting

	}else
	{
		//this part will code later . ( interrupt mode)
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT )
		{
			//1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT )
		{
			//1 . configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT )
		{
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		//3 . enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPtype << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
	}

}


/*************************************************************************************
 * @function			- GPIO_DeInit
 *
 * @brief				- This function resets the GPIO to default register
 *
 * @param[in]			- Base address of the GPIO port
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}


/*************************************************************************************
 * @function			- GPIO_ReadFromInputPin
 *
 * @brief				- This function reads the value from the Pin in the argument
 *
 * @param[in]			- Base address of the GPIO port
 * @param[in]			- Pin Number
 *
 * @return				- Value from the Pin
 *
 * @note				- none
 *************************************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);				//Check the Input Data Register. Shift to 0th bit and mask everything with zero to read the 0th bit.
	return value;
}


/*************************************************************************************
 * @function			- GPIO_ReadFromInputPort
 *
 * @brief				- This function reads the value from all Pins in the PORT
 *
 * @param[in]			- Base address of the GPIO port
 *
 * @return				- Value reads from the PORT
 *
 * @note				- none
 *************************************************************************************/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}


/*************************************************************************************
 * @function			- GPIO_WriteToOutputPin
 *
 * @brief				- This function write the value to Pins in the PORT
 *
 * @param[in]			- Base address of the GPIO port
 * @param[in]			- Pin Number
 * @param[in]			- State the Value of output
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*************************************************************************************
 * @function			- GPIO_WriteToOutputPort
 *
 * @brief				- This function write the value to all pins in the PORT
 *
 * @param[in]			- Base address of the GPIO port
 * @param[in]			- State the Value of output POrt
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/


void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*************************************************************************************
 * @function			- GPIO_ToggleOutputPin
 *
 * @brief				- This function acts as a switch to on and off the outpin pin
 *
 * @param[in]			- Base address of the GPIO port
 * @param[in]			- Pin Number
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*************************************************************************************
 * @function			- GPIO_IRQConfig
 *
 * @brief				- This function acts as a switch to on and off the outpin pin
 *
 * @param[in]			- Base address of the GPIO port
 * @param[in]			- Pin Number
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/
//IRQ Configuration & Handlers
void GPIO_IRQInterruptConfig(uint8_t IRQNumbers, uint8_t EnorDi )
{

	if(EnorDi == ENABLE)
	{
		if (IRQNumbers <= 31)
		{
			//ISER0
			*NVIC_ISER0 |= (1 << IRQNumbers);
		}
		else if (IRQNumbers > 31 && IRQNumbers < 64)
		{
			//ISER1
			*NVIC_ISER1 |= (1 << (IRQNumbers % 32) );
		}
		else if (IRQNumbers >= 64 && IRQNumbers <96)
		{
			//ISER2
			*NVIC_ISER2 |= (1 << (IRQNumbers % 64) );
		}
	}else
	{
		if (IRQNumbers <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumbers);
		}
		else if (IRQNumbers > 31 && IRQNumbers < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumbers % 32) );
		}
		else if (IRQNumbers >= 64 && IRQNumbers <96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumbers % 64) );
		}
	}
}


void GPIO_IRQPriorityConfig (uint8_t IRQNumbers, uint32_t IRQPriority)
{
	uint8_t iprx= IRQNumbers/4;
	uint8_t iprx_section = IRQNumbers % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}
/*************************************************************************************
 * @function			-
 *
 * @brief				-
 *
 * @param[in]			-
 * @param[in]			-
 * @param[in]			-
 *
 * @return				-
 *
 * @note				-
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);													//clear the pending event from exti line
	}
}
