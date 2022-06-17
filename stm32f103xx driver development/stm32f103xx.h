/*
 * stm32f103xx.h
 *
 *  Created on: Dec 21, 2021
 *      Author: shubham
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include "stdint.h"

#define __vol volatile

/* Memory addresses */
#define FLASH_BASEADDR 		0x08000000U
#define SRAM_BASEADDR 		0x20000000U

/* Bus addresses */
#define AHB1_BASEADDR 		0x40014000U
#define AHB2_BASEADDR		0x40028000U
#define APB1_BASEADDR		FLASH_BASEADDR
#define APB2_BASEADDR		0x40010000U


/* AHB1 peripheral addresses */
#define RCC_BASEADDR 		0x40021000U
#define DMA1_BASEADDR 		0x40020000U
#define DMA2_BASEADDR 		0x40020400U
#define CRC_BASEADDR 		0x40023000U
#define SDIO_BASEADDR 		0x40018000U

/* AHB2 peripheral addresses */
#define OTG_BASEADDR 		0x50000000U
#define ETHERNET_BASEADDR 	0x40028000U


/* APB2 peripheral addresses */
#define GPIOA_BASEADDR 		0x40010800U
#define GPIOB_BASEADDR 		0x40010C00U
#define GPIOC_BASEADDR 		0x40011000U
#define GPIOD_BASEADDR 		0x40011400U
#define GPIOE_BASEADDR 		0x40011800U
#define GPIOF_BASEADDR 		0x40011C00U
#define GPIOG_BASEADDR 		0x40012000U

#define EXTI_BASEADDR		0x40010400U
#define AFIO_BASEADDR		0x40010000U

#define SPI1_BASEADDR		0x40013000U
#define USART1_BASEADDR		0x40013800U

#define ADC1_BASEADDR		0x40012400U
#define ADC2_BASEADDR		0x40012800U
#define ADC3_BASEADDR		0x40013C00U

#define TIM1_BASEADDR		0x40012C00U
#define TIM8_BASEADDR		0x40013400U

/* APB1 peripheral addresses */

#define TIM2_BASEADDR		0x40000000U
#define TIM3_BASEADDR		0x40000400U
#define TIM4_BASEADDR		0x40000800U
#define TIM5_BASEADDR		0x40000C00U
#define TIM6_BASEADDR		0x40001000U
#define TIM7_BASEADDR		0x40001400U

#define RTC_BASEADDR		0x40002800U

#define SPI2_BASEADDR 		0x40003800U
#define SPI3_BASEADDR		0x40003C00U

#define UART4_BASEADDR		0x40004C00U
#define UART5_BASEADDR		0x40005000U
#define USART2_BASEADDR		0x40004400U
#define USART3_BASEADDR		0x40004800U

#define I2C1_BASEADDR		0x40005400U
#define I2C2_BASEADDR		0x40005800U

#define CAN1_BASEADDR		0x40006400U
#define CAN2_BASEADDR		0x40006800U

#define DAC_BASEADDR		0x40007400U


#define RCC 	  ((rcc_reg *)RCC_BASEADDR)					// USE BRACKETS WISELY
#define GPIOA	  ((gpio_reg *)GPIOA_BASEADDR)
#define GPIOB	  ((gpio_reg *)GPIOB_BASEADDR)
#define GPIOC	  ((gpio_reg *)GPIOC_BASEADDR)
#define GPIOD	  ((gpio_reg *)GPIOD_BASEADDR)
#define GPIOE	  ((gpio_reg *)GPIOE_BASEADDR)
#define GPIOF	  ((gpio_reg *)GPIOF_BASEADDR)
#define GPIOG	  ((gpio_reg *)GPIOG_BASEADDR)

/*  Peripheral Register definition structures */

/* GPIO register structure */
typedef struct
{
	__vol uint32_t 	CR; 		/* 			address offset : 0x00 */
	__vol uint32_t 	CFGR;		/* 			address offset : 0x04 */
	__vol uint32_t	CIR;		/* 			address offset : 0x08 */
	__vol uint32_t	APB2RSTR;	/* 			address offset : 0x0C */
	__vol uint32_t	APB1RSTR;	/* 			address offset : 0x10 */
	__vol uint32_t	AHBENR;		/* 			address offset : 0x14 */
	__vol uint32_t	APB2ENR;	/* 			address offset : 0x18 */
	__vol uint32_t	APB1ENR;	/* 			address offset : 0x1C */
	__vol uint32_t	BDCR;		/* 			address offset : 0x20 */
	__vol uint32_t	CSR;		/* 			address offset : 0x24 */
	__vol uint32_t	AHBSTR;		/* 			address offset : 0x28 */
	__vol uint32_t	CFGR2;		/* 			address offset : 0x2C */

}rcc_reg;

typedef struct
{
	__vol uint32_t 	CRL; 		/* 			address offset : 0x00 */
	__vol uint32_t 	CRH;		/* 			address offset : 0x04 */
	__vol uint32_t	IDR;		/* 			address offset : 0x08 */
	__vol uint32_t	ODR;		/* 			address offset : 0x0C */
	__vol uint32_t	BSRR;		/* 			address offset : 0x10 */
	__vol uint32_t	BRR;		/* 			address offset : 0x14 */
	__vol uint32_t	LCKR;		/* 			address offset : 0x18 */

}gpio_reg;

/* peripheral clock enable and disable macro*/

/* GPIO - peripheral clock enable */

#define GPIOA_PCLK_EN()		(RCC->APB2ENR |=  (1<<2))
#define GPIOB_PCLK_EN()		(RCC->APB2ENR |=  (1<<3))
#define GPIOC_PCLK_EN()		(RCC->APB2ENR |=  (1<<4))
#define GPIOD_PCLK_EN()		(RCC->APB2ENR |=  (1<<5))
#define GPIOE_PCLK_EN()		(RCC->APB2ENR |=  (1<<6))


#define GPIOA_PCLK_DI()		(RCC->APB2ENR &=  ~(1<<2))
#define GPIOB_PCLK_DI()		(RCC->APB2ENR &=  ~(1<<3))
#define GPIOC_PCLK_DI()		(RCC->APB2ENR &=  ~(1<<4))
#define GPIOD_PCLK_DI()		(RCC->APB2ENR &=  ~(1<<5))
#define GPIOE_PCLK_DI()		(RCC->APB2ENR &=  ~(1<<6))


#define SET		 			1
#define RESET 				0
#define ENABLE 				1
#define DISABLE 			0
#define GPIO_PIN_RESET		RESET
#define GPIO_PIN_SET		SET

#endif /* INC_STM32F103XX_H_ */
