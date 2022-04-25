/*
 * stm32f407_spi_driver.c
 *
 *  Created on: 18 Jan 2022
 *      Author: USER
 */
#include "stm32f407_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


/*************************************************************************************
 * @function			- SPI_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for given SPI
 *
 * @param[in]			- Base address of the SPI peripheral
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/



void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*************************************************************************************
 * @function			- GPIO_Init and SPI_DeInit
 *
 * @brief				- This function takes the argument base address (either it is the SPI
 * 						  or Register Address for SPI_Handle such as CPHA. CPOL, SSM, DFF, etc) by dereferencing
 * 						  data type SPI_PinConfig or pSPIx.
 *
 * @param[in]			- Base address of SPI Address or SPI Register to Set Handlers (CPHA. CPOL, SSM, DFF, etc)
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/


void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	//enable peripheral clock

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempreg = 0;

	/***************************************************************************************/
	//Set the 2nd bit - MODE of SPI

	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;


	/***************************************************************************************/
	//Set the 15th bit - BusConfig. If Simplex RX only set also 10th bit

	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//bidi mode is to be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//bidi mode to select 1
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//bidi clear
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//rxonly set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}


	/***************************************************************************************/
		//Set the 3rd to 5th bit - Set Clock Speed.
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	/***************************************************************************************/
		//Set the 11th bit - DFF

	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	/***************************************************************************************/
		//Set the 1st bit - CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	/***************************************************************************************/
		//Set the 0th bit - CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	/***************************************************************************************/
		//Set the 9th bit - SSM
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;
}



/*************************************************************************************
 * @function			- SPI_DeInit
 *
 * @brief				- This function takes the argument base address (either it is the SPI
 * 						  or Register Address for SPI_Handle such as CPHA. CPOL, SSM, DFF, etc) by dereferencing
 * 						  data type SPI_PinConfig or pSPIx.
 *
 * @param[in]			- Base address of SPI Address or SPI Register to Set Handlers (CPHA. CPOL, SSM, DFF, etc)
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}

}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*************************************************************************************
 * @function			- SPI_SendData & Receive
 *
 * @brief				- This function is used to send data. Basically it picks up the data in the TX Buffer
 *
 * @param[in]			- Pointer to the SPI
 * @param[in]			- Pointer to the TX data
 * @param[in]			- Length of the data
 *
 * @return				- none
 *
 * @note				- This is blocking call
 *************************************************************************************/


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)			//blocking API
{
	while(Len>0)
	{
		//whileTXE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit frame
			//1. Load the data in to DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}


/*************************************************************************************
 * @function			- Interrupt Settings
 *
 * @brief				- This function enables or disables interrupt and takes the IRQ number to set which EXTI line will be used
 *
 * @param[in]			- IRQ Numbers
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/


void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len>0)
	{
		//while RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		//check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit frame
			//1. Load the data in to DR
			 *((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else
		{
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}
}


/*************************************************************************************
 * @function			- Interrupt Settings
 *
 * @brief				- This function enables or disables interrupt and takes the IRQ number to set which EXTI line will be used
 *
 * @param[in]			- IRQ Numbers
 * @param[in]			- ENABLE or DISABLE macros
 *
 * @return				- none
 *
 * @note				- none
 *************************************************************************************/

void SPI_PeripheralControl (SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
		{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
		} else
		{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
		}

}

void SPI_IRQInterruptConfig(uint8_t IRQNumbers, uint8_t EnorDi )
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


void SPI_IRQPriorityConfig(uint8_t IRQNumbers, uint32_t IRQPriority)
{
	uint8_t iprx= IRQNumbers/4;
	uint8_t iprx_section = IRQNumbers % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	//1. Save the Tx buffer address and len information in some global variables
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;

	//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI Peripheral untill transmission is over
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE Flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	//4. Data transmission will be handled by ISR code
	}
	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
	//1. Save the Tx buffer address and len information in some global variables
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;

	//2. Mark the SPI state as busy in transmission so that no other code can take over same SPI Peripheral untill transmission is over
	pSPIHandle->RxState = SPI_BUSY_IN_RX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE Flag is set in SR
	pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	//4. Data transmission will be handled by ISR code
	}
	return state;
}


void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//1. check why interrupt happens. Check for TXE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);

	}
	//2. check why interrupt happens. Check for RXNE
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_rxne_interrupt_handle(pHandle);

	}

	//3. check why interrupt happens. Check for OVR flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);

	}
}


//helperfx
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
		if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit frame
			//1. Load the data in to DR
			pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}else
		{
			pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}

		if(! pSPIHandle->TxLen)
		{
			//if TxLen is zero, close the SPI comm. inform the application i sover
			//tis prevent interrupt from TXE flag
			SPI_CloseTransmission(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
		}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//check the DFF bit in CR1
		if(pSPIHandle->pSPIx->CR1 & (1 << 11) )
		{
			//16 bit frame
			//1. Load the data in to DR
			*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			(uint16_t*)pSPIHandle->pRxBuffer--;
			(uint16_t*)pSPIHandle->pRxBuffer--;
		}else
		{
			*pSPIHandle->pTxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRxBuffer--;
		}

		if(! pSPIHandle->RxLen)
		{
			//if TxLen is zero, close the SPI comm. inform the application i sover
			//tis prevent interrupt from TXE flag
			SPI_CloseReception(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//clear the ovr flag
	uint8_t temp;
	if (pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}


void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv){
	//this is weak function, application may override this function
}
