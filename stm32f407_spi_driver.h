/*
 * stm32f407_spi_driver.h
 *
 *  Created on: 18 Jan 2022
 *      Author: USER
 */

#ifndef INC_STM32F407_SPI_DRIVER_H_
#define INC_STM32F407_SPI_DRIVER_H_

#include <stm32f407xx.h>

typedef struct {
	uint8_t SPI_DeviceMode;				//Possible value from @SPI_DEVICE_MODE
	uint8_t SPI_BusConfig;				//Possible value from @
	uint8_t SPI_SclkSpeed;				//Possible value from @
	uint8_t SPI_DFF;					//Possible value from @
	uint8_t SPI_CPOL;					//Possible value from @
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;					//SSM will define the software or hardware to pull to GND the CS pin
}SPI_Config_t;

//GPIO Handler

typedef struct{
	SPI_RegDef_t 	*pSPIx; 				//This holds the base address of the SPI port to which the pin belongs
	SPI_Config_t 	SPI_Config;				//This holds SPI pin configuration settings
	uint8_t 		*pTxBuffer;				//To store the application Tx buffer address
	uint8_t			*pRxBuffer;				//To store the application Rx buffer address
	uint32_t		TxLen;					//To store Tx len
	uint32_t 		RxLen;					//To store Rx len
	uint8_t 		TxState;				//To store Tx state
	uint8_t			RxState;				//To store Tx state
}SPI_Handle_t;



//SPI Device Mode: SPI_DEVICE_MODE
#define SPI_DEVICE_MODE_MASTER						1
#define SPI_DEVICE_MODE_SLAVE						0

//SPI BUS CONFIGURATION: Full Duplex, Half Duplex or Simplex
#define SPI_BUS_CONFIG_FD							1
#define SPI_BUS_CONFIG_HD							2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY				3

//SPI CLOCK SPEED. Will be divided by Source Clock
#define SPI_SCLK_SPEED_DIV2							0
#define SPI_SCLK_SPEED_DIV4							1
#define SPI_SCLK_SPEED_DIV8							2
#define SPI_SCLK_SPEED_DIV16						3
#define SPI_SCLK_SPEED_DIV32						4
#define SPI_SCLK_SPEED_DIV64						5
#define SPI_SCLK_SPEED_DIV128						6
#define SPI_SCLK_SPEED_DIV256						7

//SPI shift Register bits
#define SPI_DFF_8BITS								0
#define SPI_DFF_16BITS								1

//SPICPOL
#define SPI_CPOL_LOW								0
#define SPI_CPOL_HIGH								1

//SPI CPHA
#define SPI_CPHA_LOW								0
#define SPI_CPHA_HIGH								1

//SPI SSM
#define SPI_SSM_EN									1
#define SPI_SSM_DI									0

#define SPI_TXE_FLAG								(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG								(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG								(1 << SPI_SR_BSY)

#define SPI_READY									0
#define SPI_BUSY_IN_RX								1
#define SPI_BUSY_IN_TX								2

//possible SPI Application Event
#define SPI_EVENT_TX_CMPLT							1
#define SPI_EVENT_RX_CMPLT							2
#define SPI_EVENT_OVR_ERR							3
#define SPI_EVENT_CRC_ERR							4

/*********************************************************************************************************************/
//SPI Peripheral Setup

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*********************************************************************************************************************/
//Init and De-init

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*********************************************************************************************************************/
//Data Send and Receive
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

//Data Send annd Receive During Interrupt
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
/*********************************************************************************************************************/
//IRQ Config and ISR Handling

void SPI_IRQInterruptConfig(uint8_t IRQNumbers, uint8_t EnorDi );
void SPI_IRQPriorityConfig(uint8_t IRQNumbers, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

void SPI_PeripheralControl (SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

#endif /* INC_STM32F407_SPI_DRIVER_H_ */
