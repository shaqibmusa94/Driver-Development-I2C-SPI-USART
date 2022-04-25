/*
 * stm32f407xx.h
 *
 *  Created on: Jan 13, 2022
 *      Author: USER
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include<stddef.h>
#include<stdint.h>


#define __vo							volatile
#define __weak							__attribute__((weak))
/*************************************************************************************************************/
//ARM Cortex Mx Processor NVIC ISER Address

#define NVIC_ISER0						((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1						((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2						((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3						((__vo uint32_t*)0xE000E10cU)

//ARM Cortex Mx Processor NVIC ICER Address

#define NVIC_ICER0						((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1						((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2						((__vo uint32_t*)0XE000E188U)
#define NVIC_ICER3						((__vo uint32_t*)0XE000E18cU)

#define NVIC_PR_BASEADDR				((__vo uint32_t*)0xE000E400U)


/***************************************************************************************************************/

#define NO_PR_BITS_IMPLEMENTED			4

#define IRQ_NO_EXTI0					6
#define IRQ_NO_EXTI1					7
#define IRQ_NO_EXTI2					8
#define IRQ_NO_EXTI3					9
#define IRQ_NO_EXTI4					10
#define IRQ_NO_EXTI9_5					23
#define IRQ_NO_EXTI15_10				40

#define IRQ_NO_SPI1						35
#define IRQ_NO_SPI2						36
#define IRQ_NO_SPI3						51

#define IRQ_NO_I2C1_EV					31
#define IRQ_NO_I2C1_ER					32
#define IRQ_NO_I2C2_EV					33
#define IRQ_NO_I2C2_ER					34
#define IRQ_NO_I2C3_EV					72
#define IRQ_NO_I2C3_ER					73

#define NVIC_IRQ_PRI0					0
#define NVIC_IRQ_PRI1					1
#define NVIC_IRQ_PRI2					2
#define NVIC_IRQ_PRI3					3
#define NVIC_IRQ_PRI4					4
#define NVIC_IRQ_PRI5					5
#define NVIC_IRQ_PRI6					6
#define NVIC_IRQ_PRI7					7
#define NVIC_IRQ_PRI8					8
#define NVIC_IRQ_PRI9					9
#define NVIC_IRQ_PRI10					10
#define NVIC_IRQ_PRI11					11
#define NVIC_IRQ_PRI12					12
#define NVIC_IRQ_PRI13					13
#define NVIC_IRQ_PRI14					14
#define NVIC_IRQ_PRI15					15

/*******************************************************************************/
//BASEADDRESSES

#define FLASH_BASEADDR					0x08000000U
#define SRAM1_BASEADDR					0x20000000U
#define SRAM2_BASEADDR					0x20001C00U
#define ROM_BASEADDR					0x1FFF0000U
#define SRAM							SRAM1_BASEADDR

#define APB1_PERIPH_BASEADDR			0x40000000U
#define APB2_PERIPH_BASEADDR			0x40010000U
#define AHB1_PERIPH_BASEADDR			0x40020000U
#define AHB2_PERIPH_BASEADDR			0x50000000U
#define RCC_BASEADDR					0x40023800U


/********************************************************************************/
//BASEADDRESSES PLUS WITH OFFSET

//GPIO
#define GPIOA_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR					(AHB1_PERIPH_BASEADDR + 0x2000)

//I2C
#define I2C1_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5C00)

//SPI
#define SPI1_BASEADDR					(APB2_PERIPH_BASEADDR + 0x3000)
#define SPI2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR					(APB1_PERIPH_BASEADDR + 0x3C00)

//USART
#define USART1_BASEADDR					(APB2_PERIPH_BASEADDR + 0x1000)
#define USART2_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4800)
#define USART6_BASEADDR					(APB2_PERIPH_BASEADDR + 0x1400)

//UART
#define UART4_BASEADDR					(APB1_PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR					(APB1_PERIPH_BASEADDR + 0x5000)

//EXTI
#define EXTI_BASEADDR					(APB2_PERIPH_BASEADDR + 0x3C00)

//SYSCFG
#define SYSCFG_BASEADDR					(APB2_PERIPH_BASEADDR + 0x3800)

//ADC
#define ADC_BASEADDR					(APB2_PERIPH_BASEADDR + 0x2000)

/***************************************************************************************************/
//PERIPHERAL REGISTER ADDR - RegDef
// variables are created as a place holder to hold the values
//registers of a peripheral are specific to MCU

//GPIO RegDef
typedef struct{
	volatile uint32_t MODER;			//GPIO port mode register
	volatile uint32_t OTYPER;			//GPIO port output type register
	volatile uint32_t OSPEEDR;			//GPIO port output speed register
	volatile uint32_t PUPDR;			//GPIO port pull-up/pull-down register
	volatile uint32_t IDR;				//GPIO port input data register
	volatile uint32_t ODR;				//GPIO port output data register
	volatile uint32_t BSRR;				//GPIO port bit set/reset register
	volatile uint32_t LCKR;				//GPIO port configuration lock register
	volatile uint32_t AFR[2];			//GPIO alternate function low then high register
}GPIO_RegDef_t;


//RCC RegDef
typedef struct {
	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	volatile uint32_t Reserved1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t Reserved2;
	volatile uint32_t Reserved3;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t Reserved4;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	volatile uint32_t Reserved5;
	volatile uint32_t Reserved6;
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t Reserved7;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	volatile uint32_t Reserved8;
	volatile uint32_t Reserved9;
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	volatile uint32_t Reserved10;
	volatile uint32_t Reserved11;
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
}RCC_RegDef_t;


//EXTI RegDef
typedef struct{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
}EXTI_RegDef_t;


//SYSCFG RegDef
typedef struct{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	volatile uint32_t RESERVED2[2];
	volatile uint32_t CFGR;
}SYSCFG_RegDef_t;


//SPI RegDef
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2SCFGR;
	volatile uint32_t I2SPR;
}SPI_RegDef_t;

//I2C RegDef
typedef struct{
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;
}I2C_RegDef_t;


//ADC RegDef

typedef struct{
	volatile uint32_t SR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SMPR1;
	volatile uint32_t SMPR2;
	volatile uint32_t JOFR1;
	volatile uint32_t JOFR2;
	volatile uint32_t JOFR3;
	volatile uint32_t JOFR4;
	volatile uint32_t HTR;
	volatile uint32_t SQR1;
	volatile uint32_t SQR2;
	volatile uint32_t SQR3;
	volatile uint32_t JSQR;
	volatile uint32_t JDR1;
	volatile uint32_t JDR2;
	volatile uint32_t JDR3;
	volatile uint32_t JDR4;
	volatile uint32_t DR;
}ADC_RegDef_t;


typedef struct{
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t BRR;
	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t CR3;
	volatile uint32_t GTPR;
}USART_RegDef_t;

/***********************************************************************************************************/
//HOLDER OF BASEADDRESS TO ACCESS REGDEF ELEMENTS


#define GPIOA								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF								((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG								((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH								((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI								((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI								((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG								((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1								((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2								((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3								((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1								((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2								((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3								((I2C_RegDef_t*)I2C3_BASEADDR)

#define ADC									((ADC_RegDef_t*)ADC_BASEADDR)

#define USART1								((USART_RegDef_t*)USART1_BASEADDR)
#define USART2								((USART_RegDef_t*)USART2_BASEADDR)
#define USART3								((USART_RegDef_t*)USART3_BASEADDR)
#define USART6								((USART_RegDef_t*)USART6_BASEADDR)
#define UART4								((USART_RegDef_t*)UART4_BASEADDR)
#define UART5								((USART_RegDef_t*)UART5_BASEADDR)

/*****************************************************************************************************/
//ENABLING AND DISABLING ALL THE PERIPHERAL CLOCK AVAILABLE IN STM32F407

//Enable peripheral clock for GPIO Peripheral
#define GPIOA_PCLK_EN()						(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()						(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()						(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()						(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()						(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()						(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()						(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()						(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()						(RCC->AHB1ENR |= (1 << 8))

//Enable peripheral clock for I2C
#define I2C1_PCLK_EN()						(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()						(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()						(RCC->APB1ENR |= (1 << 23))

//Enable peripheral clock for SPI
#define SPI1_PCLK_EN()						(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()						(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()						(RCC->APB1ENR |= (1 << 15))

//Enable peripheral clock for USART
#define USART1_PCLK_EN()					(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()					(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()					(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()						(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()						(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()					(RCC->APB2ENR |= (1 << 5))

//Enable SYSCFG Periperal Clock
#define SYSCFG_PCLK_EN()					(RCC->APB2ENR |= (1 << 14))

//Enable ADC Peripheral Clock
#define ADC_PCLK_EN()						(RCC->APB2ENR |= (1 << 8))

//Enable USART Peripheral Clock
#define USART1_PCLK_EN()					(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()					(RCC->APB2ENR |= (1 << 5))
#define USART2_PCLK_EN()					(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()					(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()						(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()						(RCC->APB1ENR |= (1 << 20))
/*****************************************************************************************************/
//Disable peripheral clock GPIO

#define GPIOA_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()						(RCC->AHB1ENR &= ~(1 << 8))

//Disable peripheral clock for I2C
#define I2C1_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 23))

//Disable peripheral clock for SPI
#define SPI1_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 15))

//Disable peripheral clock for USART
#define USART1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()						(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 5))

//Disable ADC peripheral clock
#define ADC_PCLK_DI()						(RCC->APB2ENR &= ~(1 << 8))


//Reset the GPIO
#define GPIOA_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET()					do {(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)

//Reset the SPI
#define SPI1_REG_RESET()					do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));}while(0)
#define SPI2_REG_RESET()					do {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));}while(0)
#define SPI3_REG_RESET()					do {(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));}while(0)

//Reset the I2C
#define I2C1_REG_RESET()					do {(RCC->APB1ENR |= (1 << 21)); (RCC->APB1ENR &= ~(1 << 21));}while(0)
#define I2C2_REG_RESET()					do {(RCC->APB1ENR |= (1 << 22)); (RCC->APB1ENR &= ~(1 << 22));}while(0)
#define I2C3_REG_RESET()					do {(RCC->APB1ENR |= (1 << 23)); (RCC->APB1ENR &= ~(1 << 23));}while(0)

//Reset the USART
#define USART1_REG_RESET()					do {(RCC->APB2ENR |= (1 << 4)); (RCC->APB2ENR &= ~(1 << 4));}while(0)
#define USART2_REG_RESET()					do {(RCC->APB1ENR |= (1 << 17)); (RCC->APB1ENR &= ~(1 << 17));}while(0)
#define USART3_REG_RESET()					do {(RCC->APB1ENR |= (1 << 18)); (RCC->APB1ENR &= ~(1 << 18));}while(0)
#define USART6_REG_RESET()					do {(RCC->APB2ENR |= (1 << 5)); (RCC->APB2ENR &= ~(1 << 5));}while(0)
#define UART4_REG_RESET()					do {(RCC->APB1ENR |= (1 << 19)); (RCC->APB1ENR &= ~(1 << 19));}while(0)
#define UART5_REG_RESET()					do {(RCC->APB1ENR |= (1 << 20)); (RCC->APB1ENR &= ~(1 << 20));}while(0)



#define GPIO_BASEADDR_TO_CODE(x)			((x == GPIOA)? 0 :\
											(x == GPIOB)? 1 :\
											(x == GPIOC)? 2 :\
											(x == GPIOD)? 3 :\
											(x == GPIOE)? 4 :\
											(x == GPIOF)? 5 :\
											(x == GPIOG)? 6 :\
											(x == GPIOH)? 7 :\
											(x == GPIOI)? 8 :0)


#define ENABLE 								1
#define DISABLE 							0
#define SET 								ENABLE
#define RESET 								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET
#define FLAG_RESET							RESET
#define FLAG_SET							SET

/********************************************************************************************/
//BIT POSITION IN REGISTER

//REGISTER: SPI_CR1
#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BR							3
#define SPI_CR1_SPE							6
#define SPI_CR1_LSBFIRST					7
#define SPI_CR1_SSI							8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_DFF							11
#define SPI_CR1_CRCNEXT						12
#define SPI_CR1_CRCEN						13
#define SPI_CR1_BIDIOE						14
#define SPI_CR1_BIDIMODE					15

//REGISTER: SPI_CR2
#define SPI_CR2_RXDMAEN						0
#define SPI_CR2_TXDMAEN						1
#define SPI_CR2_SSOE						2
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE						6
#define SPI_CR2_TXEIE						7

//REGISTER: SPI_SR
#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRC_ERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8

//Register I2C_CR1
#define I2C_CR1_PE							0
#define I2C_CR1_SMBUS						1
#define I2C_CR1_Res1						2
#define I2C_CR1_SMBTYPE						3
#define I2C_CR1_ENARP						4
#define I2C_CR1_ENPEC						5
#define I2C_CR1_ENGC						6
#define I2C_CR1_NOSTRETCH					7
#define I2C_CR1_START						8
#define I2C_CR1_STOP						9
#define I2C_CR1_ACK							10
#define I2C_CR1_POS							11
#define I2C_CR1_PEC							12
#define I2C_CR1_ALERT						13
#define I2C_CR1_Res2						14
#define I2C_CR1_SWRST						15

//Register I2C_CR2
#define I2C_CR2_FREQ						0
#define I2C_CR2_ITERREN						8
#define I2C_CR2_ITEVTEN						9
#define I2C_CR2_ITBUFEN						10
#define I2C_CR2_DMAEN						11
#define I2C_CR2_LAST						12

//Register I2C_OAR1
#define I2C_OAR1_ADD0						0
#define I2C_OAR1_ADD1_7						1
#define I2C_OAR1_ADD8_9						8
#define I2C_OAR1_ADDMODE					15

//Register I2C_OAR2
#define I2C_OAR2_ENDUAL						0
#define I2C_OAR2_ADD2						1

//Register I2C_DR
#define I2C_DR_DR							0

//Register I2C_SR1
#define I2C_SR1_SB							0
#define I2C_SR1_ADDR						1
#define I2C_SR1_BTF							2
#define I2C_SR1_ADD10						3
#define I2C_SR1_STOPF						4
#define I2C_SR1_Res1						5
#define I2C_SR1_RXNE						6
#define I2C_SR1_TXE							7
#define I2C_SR1_BERR						8
#define I2C_SR1_ARLO						9
#define I2C_SR1_AF							10
#define I2C_SR1_OVR							11
#define I2C_SR1_PECERR						12
#define I2C_SR1_Res2						13
#define I2C_SR1_TIMEOUT						14
#define I2C_SR1_SMBALERT					15

//Register I2C_SR2
#define I2C_SR2_MSL							0
#define I2C_SR2_BUSY						1
#define I2C_SR2_TRA							2
#define I2C_SR2_Res1						3
#define I2C_SR2_GENCALL						4
#define I2C_SR2_SMBDEFAULT					5
#define I2C_SR2_SMBHOST						6
#define I2C_SR2_DUALF						7
#define I2C_SR2_PEC							8

//Register I2C_CCR
#define I2C_CCR_CCR							0
#define I2C_CCR_DUTY						14
#define I2C_CCR_FS							15

//Register I2C_TRISE
#define I2C_TRISE_TRISE						0

//Register I2C_FLTR
#define I2C_FLTR_DNF						0
#define I2C_FLTR_ANOFF						4

//Register USART_SR
#define USART_SR_PE							0
#define USART_SR_FE							1
#define USART_SR_NF							2
#define USART_SR_ORE						3
#define USART_SR_IDLE						4
#define USART_SR_RXNE						5
#define USART_SR_TC							6
#define USART_SR_TXE						7
#define USART_SR_LBD						8
#define USART_SR_CTS						9

//Register USART_CR1
#define USART_CR1_SBK						0
#define USART_CR1_RWU						1
#define USART_CR1_RE						2
#define USART_CR1_TE						3
#define USART_CR1_IDLEIE					4
#define USART_CR1_RXNEIE					5
#define USART_CR1_TCIE						6
#define USART_CR1_TXEIE						7
#define USART_CR1_PEIE						8
#define USART_CR1_PS						9
#define USART_CR1_PCE						10
#define USART_CR1_WAKE						11
#define USART_CR1_M							12
#define USART_CR1_UE						13
#define USART_CR1_OVER8						15

//Register USART_CR2
#define USART_CR2_ADD						0
#define USART_CR2_LBDL						5
#define USART_CR2_LBDIE						6
#define USART_CR2_LBCL						8
#define USART_CR2_CPHA						9
#define USART_CR2_CPOL						10
#define USART_CR2_CLKEN						11
#define USART_CR2_STOP						12
#define USART_CR2_LINEN						14

//Register USART_CR3
#define USART_CR3_EIE						0
#define USART_CR3_IREN						1
#define USART_CR3_IRLP						2
#define USART_CR3_HDSEL						3
#define USART_CR3_NACK						4
#define USART_CR3_SCEN						5
#define USART_CR3_DMAR						6
#define USART_CR3_DMAT						7
#define USART_CR3_RTSE						8
#define USART_CR3_CTSE						9
#define USART_CR3_CTSIE						10
#define USART_CR3_ONEBIT					11


#include "stm32f407_gpio_driver.h"
#include "stm32f407_spi_driver.h"
#include "stm32f407_i2c_driver.h"
#include "stm32f407_adc_driver.h"
#include "stm32f407_usart_driver.h"
#include "stm32f407_rcc_driver.h"

#endif /* INC_STM32F407XX_H_ */
