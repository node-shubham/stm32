/*
 * stm32f401xx.h
 *
 *  Created on: Dec 18, 2021
 *      Author: Shubham saha ray
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include "stdint.h"
#include "stddef.h"

#define __vol volatile
#define __weak  __attribute__((weak))

/* processor specific registers */

#define NVIC_ISER0		( (__vol uint32_t *)0xE000E100 )
#define NVIC_ISER1		( (__vol uint32_t *)0xE000E104 )
#define NVIC_ISER2		( (__vol uint32_t *)0xE000E108 )
#define NVIC_ISER3		( (__vol uint32_t *)0xE000E10C )

#define NVIC_ICER0		( (__vol uint32_t *)0XE000E180 )
#define NVIC_ICER1		( (__vol uint32_t *)0xE000E184 )
#define NVIC_ICER2		( (__vol uint32_t *)0xE000E188 )
#define NVIC_ICER3		( (__vol uint32_t *)0xE000E18C )

#define NVIC_IPR_BASEADDR 	( (__vol uint32_t *)0xE000E400 )

#define NO_IPR_BITS 			4

/* MCU specific registers and addresses */

#define FLASH_BASE_ADDR			0x08000000U
#define ROM_BASE_ADDR			0x1FFF0000U
#define SRAM_BASE_ADDR			0x20000000U
#define SRAM					SRAM_BASE_ADDR

/* BUS Addresses */

#define PERIPH_BASE_ADDR		0x40000000U
#define APB1_BASE_ADDR			PERIPH_BASE_ADDR
#define APB2_BASE_ADDR			0x40010000U
#define AHB1_BASE_ADDR			0x40020000U
#define AHB2_BASE_ADDR			0x50000000U

/* AHB1 peripheral addresses */

#define GPIOA_BASE_ADDR			(AHB1_BASE_ADDR + 0X0000)   // BASE_ADDR + ADD OFSET
#define GPIOB_BASE_ADDR			(AHB1_BASE_ADDR + 0X0400)
#define GPIOC_BASE_ADDR			(AHB1_BASE_ADDR + 0X0800)
#define GPIOD_BASE_ADDR			(AHB1_BASE_ADDR + 0X0C00)
#define GPIOE_BASE_ADDR			(AHB1_BASE_ADDR + 0X1000)
#define GPIOH_BASE_ADDR			(AHB1_BASE_ADDR + 0X1C00)

#define RCC_BASE_ADDR			(AHB1_BASE_ADDR + 0X3800)	// FEW PERIPH LEFT ADD IT LATER


/* APB1 peripheral addresses */

#define TIM2_BASE_ADDR			(APB1_BASE_ADDR + 0X0000)

#define I2C1_BASE_ADDR			(APB1_BASE_ADDR + 0X5400)
#define I2C2_BASE_ADDR			(APB1_BASE_ADDR + 0X5800)
#define I2C3_BASE_ADDR			(APB1_BASE_ADDR + 0X5C00)

#define SPI2_BASE_ADDR			(APB1_BASE_ADDR + 0X3800)
#define SPI3_BASE_ADDR			(APB1_BASE_ADDR + 0X3C00)

#define USART2_BASE_ADDR		(APB1_BASE_ADDR + 0X4400)

/* APB2 peripheral addresses */

#define TIM1_BASE_ADDR			(APB2_BASE_ADDR + 0X0000)

#define SPI1_BASE_ADDR			(APB2_BASE_ADDR + 0X3000)
#define SPI4_BASE_ADDR			(APB2_BASE_ADDR + 0X3400)

#define USART1_BASE_ADDR		(APB2_BASE_ADDR + 0X1000)
#define USART6_BASE_ADDR		(APB2_BASE_ADDR + 0X1400)

#define SYSCFG_BASE_ADDR		(APB2_BASE_ADDR + 0X3800)
#define EXTI_BASE_ADDR			(APB2_BASE_ADDR + 0X3C00)

/* AHB2 peripheral addresses */

#define OTG_BASE_ADDR			(AHB2_BASE_ADDR + 0X0000)


/*  Peripheral Register definition structures */

/* GPIO register structure */
typedef struct
{
	__vol uint32_t MODER; 			//	GPIO port mode register        			( Address offset : 0x00 )
	__vol uint32_t OTYPER; 			//	GPIO port output type register 			( Address offset : 0x04 )
	__vol uint32_t OSPEEDR; 		// 	GPIO port output speed register			( Address offset : 0x08 )
	__vol uint32_t PUPDR; 			//	GPIO port pull-up/pull-down register 	( Address offset : 0x0C )
	__vol uint32_t IDR; 			// 	GPIO port input data register 			( Address offset : 0x10 )
	__vol uint32_t ODR; 			// 	GPIO port output data register			( Address offset : 0x14 )
	__vol uint32_t BSRR; 			//	GPIO port bit set/reset register		( Address offset : 0x18 )
	__vol uint32_t LCKR; 			//	GPIO port configuration lock register	( Address offset : 0x1C )
	__vol uint32_t AFR[2]; 			//	GPIO alternate L+H function register	( Address offset : 0x20 ; 0x24 )

}gpio_reg;


/* RCC register structure */
typedef struct
{
	__vol uint32_t CR; 				//	RCC clock control register        			( Address offset : 0x00 )
	__vol uint32_t PLLCFGR; 		//	RCC PLL configuration register        		( Address offset : 0x04 )
	__vol uint32_t CFGR; 			//	RCC clock configuration register       		( Address offset : 0x08 )
	__vol uint32_t CIR; 			//	RCC clock interrupt register       			( Address offset : 0x0C )
	__vol uint32_t AHB1RSTR; 		//	RCC AHB1 peripheral reset register        	( Address offset : 0x10 )
	__vol uint32_t AHB2RSTR; 		//	RCC AHB2 peripheral reset register        	( Address offset : 0x14 )
		  uint32_t RESERVED0[2]; 	//	reserved        							( Address offset : 0x18 + 0x1C)
	__vol uint32_t APB1RSTR; 		//	RCC APB1 peripheral reset register       	( Address offset : 0x20 )
	__vol uint32_t APB2RSTR; 		//	RCC APB2 peripheral reset register       	( Address offset : 0x24 )
		  uint32_t RESERVED1[2]; 	//	reserved 									( Address offset : 0x28 + 0x2C)
	__vol uint32_t AHB1ENR; 		//	RCC AHB1 peripheral clock enable register	( Address offset : 0x30 )
	__vol uint32_t AHB2ENR; 		//	RCC AHB2 peripheral clock enable register   ( Address offset : 0x34 )
		  uint32_t RESERVED2[2];	//	reserved 									( Address offset : 0x38 + 0x3C)
	__vol uint32_t APB1ENR ; 		//	RCC APB1 peripheral clock enable register	       			( Address offset : 0x40 )
	__vol uint32_t APB2ENR; 		//	RCC APB2 peripheral clock enable register       			( Address offset : 0x44 )
		  uint32_t RESERVED3[2]; 	//	reserved 													( Address offset : 0x48 + 0X4C )
	__vol uint32_t AHB1LPENR ; 		//	RCC AHB1 peripheral clock enable in low power mode register	( Address offset : 0x50 )
	__vol uint32_t AHB2LPENR ; 		//	RCC AHB2 peripheral clock enable in low power mode register	( Address offset : 0x54 )
		  uint32_t RESERVED4[2]; 	//	reserved       												( Address offset : 0x58 + 0X5C )
	__vol uint32_t APB1LPENR; 		//	RCC APB1 peripheral clock enable in low power mode register ( Address offset : 0x60 )
	__vol uint32_t APB2LPENR ; 		//	RCC APB2 peripheral clock enable in low power mode register ( Address offset : 0x64 )
		  uint32_t RESERVED5[2]; 	//	reserved       												( Address offset : 0x68 + 0X6C )
	__vol uint32_t BDCR ;	 		//	RCC Backup domain control register       					( Address offset : 0x70 )
	__vol uint32_t CSR ; 			//	RCC clock control & status register       					( Address offset : 0x74 )
		  uint32_t RESERVED6[2]; 	//	reserved        											( Address offset : 0x78 + 0x7C )
	__vol uint32_t SSCGR ; 			//	RCC spread spectrum clock generation register       		( Address offset : 0x80 )
	__vol uint32_t PLLI2SCFGR ; 	//	RCC PLLI2S configuration register       					( Address offset : 0x84 )
		  uint32_t RESERVED7; 		//	reserved        											( Address offset : 0x88 )
	__vol uint32_t DCKCFGR ; 		//	RCC Dedicated Clocks Configuration Register       			( Address offset : 0x8C )

}rcc_reg;


/* EXTI register structure */
typedef struct
{
	__vol uint32_t IMR; 			//	Interrupt mask register        				( Address offset : 0x00 )
	__vol uint32_t EMR; 			//	Event mask register 						( Address offset : 0x04 )
	__vol uint32_t RTSR; 			// 	Rising trigger selection register			( Address offset : 0x08 )
	__vol uint32_t FTSR; 			//	Falling trigger selection register 			( Address offset : 0x0C )
	__vol uint32_t SWIER; 			// 	Software interrupt event register 			( Address offset : 0x10 )
	__vol uint32_t PR; 				// 	Pending register							( Address offset : 0x14 )

}exti_reg;


/* SYSCFG register structure */
typedef struct
{
	__vol uint32_t MEMRMP; 			//	SYSCFG memory remap register        						( Address offset : 0x00 )
	__vol uint32_t PMC; 			// 	SYSCFG peripheral mode configuration register				( Address offset : 0x04 )
	__vol uint32_t EXTICR[4]; 		// 	SYSCFG external interrupt configuration register 1			( Address offset : 0x08 )
//	__vol uint32_t EXTICR2; 		//	SYSCFG external interrupt configuration register 2 			( Address offset : 0x0C )
//	__vol uint32_t EXTICR3; 		// 	SYSCFG external interrupt configuration register 3 			( Address offset : 0x10 )
//	__vol uint32_t EXTICR4; 		// 	SYSCFG external interrupt configuration register 4			( Address offset : 0x14 )
		  uint32_t RESERVED[2]; 	//	Reserved													( Address offset : 0x18 + 0x1C)
	__vol uint32_t CMPCR; 			//	Compensation cell control register							( Address offset : 0x20 )

}syscfg_reg;


/* SPI register structure */
typedef struct
{
	__vol uint32_t CR1; 			//	SPI control register 1        			( Address offset : 0x00 )
	__vol uint32_t CR2;				//	SPI control register 2					( Address offset : 0x04 )
	__vol uint32_t SR; 				//	SPI status register						( Address offset : 0x08 )
	__vol uint32_t DR; 				// 	SPI data register						( Address offset : 0x0C )
	__vol uint32_t CRCPR; 			//	SPI CRC polynomial register				( Address offset : 0x10 )
	__vol uint32_t RXCRCR; 			// 	SPI RX CRC register						( Address offset : 0x14 )
	__vol uint32_t TXCRCR; 			// 	SPI TX CRC register						( Address offset : 0x18 )
	__vol uint32_t I2SCFGR; 		//	SPI_I2S configuration register			( Address offset : 0x1C )
	__vol uint32_t I2SPR; 			//	SPI_I2S prescaler register				( Address offset : 0x20 )

}spi_reg;


/* I2C register structure */
typedef struct
{
	__vol uint32_t CR1; 			//	I2C Control register        			( Address offset : 0x00 )
	__vol uint32_t CR2; 			//	I2C Control register        			( Address offset : 0x04 )
	__vol uint32_t OAR1; 			// 	I2C Own address register				( Address offset : 0x08 )
	__vol uint32_t OAR2; 			//	I2C Control register        			( Address offset : 0x0C )
	__vol uint32_t DR; 				// 	I2C Data register						( Address offset : 0x10 )
	__vol uint32_t SR1; 			// 	I2C Status register						( Address offset : 0x14 )
	__vol uint32_t SR2; 			// 	I2C Status register						( Address offset : 0x18 )
	__vol uint32_t CCR; 			//	I2C Clock control register				( Address offset : 0x1C )
	__vol uint32_t TRISE; 			//	I2C TRISE register						( Address offset : 0x20 )
	__vol uint32_t FLTR; 			//	I2C FLTR register						( Address offset : 0x24 )

}i2c_reg;

/* USART register structure */
typedef struct
{
	__vol uint32_t SR; 				//	 Status register     		  			( Address offset : 0x00 )
	__vol uint32_t DR; 				// 	Data register							( Address offset : 0x04 )
	__vol uint32_t BRR; 			// 	Baud rate register						( Address offset : 0x08 )
	__vol uint32_t CR1; 			//	Control register 1						( Address offset : 0x0C )
	__vol uint32_t CR2; 			//	Control register 2						( Address offset : 0x10 )
	__vol uint32_t CR3; 			//	Control register 3 						( Address offset : 0x14 )
	__vol uint32_t GTPR; 			//	Guard time and prescaler register		( Address offset : 0x18 )

}usart_reg;




/* xxx register structure
typedef struct
{
	__vol uint32_t MODER; 			//	        			( Address offset : 0x00 )
	__vol uint32_t OTYPER; 			// 			( Address offset : 0x04 )
	__vol uint32_t OSPEEDR; 		// 			( Address offset : 0x08 )
	__vol uint32_t PUPDR; 			//	 	( Address offset : 0x0C )
	__vol uint32_t IDR; 			// 	 			( Address offset : 0x10 )
	__vol uint32_t ODR; 			// 				( Address offset : 0x14 )
	__vol uint32_t BSRR; 			//			( Address offset : 0x18 )
	__vol uint32_t LCKR; 			//		( Address offset : 0x1C )
	__vol uint32_t AFR[2]; 			//		( Address offset : 0x20 ; 0x24 )

}xxx_reg;
*/

/* peripheral register definition : base addresses typecasted to gpio_reg */

#define GPIOA 	((gpio_reg *)GPIOA_BASE_ADDR)
#define GPIOB 	((gpio_reg *)GPIOB_BASE_ADDR)
#define GPIOC 	((gpio_reg *)GPIOC_BASE_ADDR)
#define GPIOD 	((gpio_reg *)GPIOD_BASE_ADDR)
#define GPIOE 	((gpio_reg *)GPIOE_BASE_ADDR)
#define GPIOH 	((gpio_reg *)GPIOH_BASE_ADDR)

/* peripheral register definition  */

#define RCC 	((rcc_reg *)RCC_BASE_ADDR)
#define EXTI 	((exti_reg *)EXTI_BASE_ADDR)
#define SYSCFG 	((syscfg_reg *)SYSCFG_BASE_ADDR)

#define SPI1	((spi_reg *)SPI1_BASE_ADDR)
#define SPI2	((spi_reg *)SPI2_BASE_ADDR)
#define SPI3	((spi_reg *)SPI3_BASE_ADDR)
#define SPI4	((spi_reg *)SPI4_BASE_ADDR)


#define I2C1	((i2c_reg *)I2C1_BASE_ADDR)
#define I2C2	((i2c_reg *)I2C2_BASE_ADDR)
#define I2C3	((i2c_reg *)I2C3_BASE_ADDR)

#define USART1	((usart_reg *)USART1_BASE_ADDR)
#define USART2	((usart_reg *)USART2_BASE_ADDR)
#define USART6	((usart_reg *)USART6_BASE_ADDR)

/* GPIO peripheral clock enable/disable macro */

#define GPIOA_PCLK_EN()   (RCC->AHB1ENR |= ( 1<< 0))   	// ENABLE
#define GPIOB_PCLK_EN()   (RCC->AHB1ENR |= ( 1<< 1))
#define GPIOC_PCLK_EN()   (RCC->AHB1ENR |= ( 1<< 2))
#define GPIOD_PCLK_EN()   (RCC->AHB1ENR |= ( 1<< 3))
#define GPIOE_PCLK_EN()   (RCC->AHB1ENR |= ( 1<< 4))
#define GPIOH_PCLK_EN()   (RCC->AHB1ENR |= ( 1<< 7))

#define GPIOA_PCLK_DI()   (RCC->AHB1ENR &= ~( 1<< 0))		// DISABLE
#define GPIOB_PCLK_DI()   (RCC->AHB1ENR &= ~( 1<< 1))
#define GPIOC_PCLK_DI()   (RCC->AHB1ENR &= ~( 1<< 2))
#define GPIOD_PCLK_DI()   (RCC->AHB1ENR &= ~( 1<< 3))
#define GPIOE_PCLK_DI()   (RCC->AHB1ENR &= ~( 1<< 4))
#define GPIOH_PCLK_DI()   (RCC->AHB1ENR &= ~( 1<< 7))


/* I2C peripheral clock enable/disable macro */

#define I2C1_PCLK_EN()   (RCC->APB1ENR |= ( 1<< 21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= ( 1<< 22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |= ( 1<< 23))

#define I2C1_PCLK_DI()   (RCC->APB1ENR &= ~( 1<< 21))
#define I2C2_PCLK_DI()   (RCC->APB1ENR &= ~( 1<< 22))
#define I2C3_PCLK_DI()   (RCC->APB1ENR &= ~( 1<< 23))


/* SPI peripheral clock enable/disable macro */

#define SPI1_PCLK_EN()   (RCC->APB2ENR |= ( 1<< 12))
#define SPI4_PCLK_EN()   (RCC->APB2ENR |= ( 1<< 13))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= ( 1<< 14))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= ( 1<< 15))

#define SPI1_PCLK_DI()   (RCC->APB2ENR &= ~( 1<< 12))
#define SPI4_PCLK_DI()   (RCC->APB2ENR &= ~( 1<< 13))
#define SPI2_PCLK_DI()   (RCC->APB1ENR &= ~( 1<< 14))
#define SPI3_PCLK_DI()   (RCC->APB1ENR &= ~( 1<< 15))

/* USART peripheral clock enable/disable macro */

#define USART1_PCLK_EN()   (RCC->APB2ENR |= ( 1<< 4))
#define USART6_PCLK_EN()   (RCC->APB2ENR |= ( 1<< 5))
#define USART2_PCLK_EN()   (RCC->APB1ENR |= ( 1<< 17))

#define USART1_PCLK_DI()   (RCC->APB2ENR &= ~( 1<< 4))
#define USART6_PCLK_DI()   (RCC->APB2ENR &= ~( 1<< 5))
#define USART2_PCLK_DI()   (RCC->APB1ENR &= ~( 1<< 17))



/* SYSCFG peripheral clock enable/disable macro */

#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |= ( 1<< 14))
#define SYSCFG_PCLK_DI()   (RCC->APB2ENR &= ~( 1<< 14))

/* macro to reset GPIOx peripheral */

#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<0) );(RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<1) );(RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<2) );(RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<3) );(RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<4) );(RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1<<7) );(RCC->AHB1RSTR &= ~(1<<7));}while(0)

/*
 *  returns port code for given GPIOx base address
 */
/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)?0:\
									 (x == GPIOB)?1:\
									 (x == GPIOC)?2:\
									 (x == GPIOD)?3:\
									 (x == GPIOE)?4:\
									 (x == GPIOH)?7:0)

#define SPI1_REG_RESET()	do{ (RCC->APB2RSTR |= (1<<12) );(RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<14) );(RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<15) );(RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()	do{ (RCC->APB2RSTR |= (1<<13) );(RCC->APB2RSTR &= ~(1<<13));}while(0)

#define I2C1_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<21) );(RCC->APB1RSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<22) );(RCC->APB1RSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<23) );(RCC->APB1RSTR &= ~(1<<23));}while(0)

#define USART1_REG_RESET()	do{ (RCC->APB2RSTR |= (1<<4) );(RCC->APB2RSTR &= ~(1<<4));}while(0)
#define USART6_REG_RESET()	do{ (RCC->APB2RSTR |= (1<<5) );(RCC->APB2RSTR &= ~(1<<5));}while(0)
#define USART2_REG_RESET()	do{ (RCC->APB1RSTR |= (1<<17) );(RCC->APB1RSTR &= ~(1<<17));}while(0)



#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34
#define IRQ_NO_I2C3_EV		72
#define IRQ_NO_I2C3_ER		73

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART6		71

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


/* some generic macros*/

#define ENABLE 				1
#define DISABLE 			0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_RESET		DISABLE
#define GPIO_PIN_SET		ENABLE
#define FLAG_RESET 			RESET
#define FLAG_SET 			SET

/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8


/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE    					 0
#define I2C_CR1_SMBUS      				 1
#define I2C_CR1_SMBTYPE   				 3
#define I2C_CR1_ENARP     				 4
#define I2C_CR1_ENPEC   			 	 5
#define I2C_CR1_ENGC     				 6
#define I2C_CR1_NOSTRETCH     			 7
#define I2C_CR1_START      		 		 8
#define I2C_CR1_STOP     			 	 9
#define I2C_CR1_ACK   			 		 10
#define I2C_CR1_POS  			 		 11
#define I2C_CR1_PEC     			 	 12
#define I2C_CR1_ALERT     				 13
#define I2C_CR1_SWRST     				 15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9




#include "f401_gpio.h"
#include "f401_rcc.h"
#include "f401_spi.h"
#include "f401_i2c.h"
#include "f401_usart.h"



#endif /* INC_STM32F401XX_H_ */
