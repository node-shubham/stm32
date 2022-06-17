/*
 * f401_gpio.h
 *
 *  Created on: 19-Dec-2021
 *      Author: shubham saha ray
 */

#ifndef INC_F401_GPIO_H_
#define INC_F401_GPIO_H_

#include "stm32f401xx.h"

typedef struct 		// config structure of pin configuration
{
	uint32_t pin_no;
	uint32_t pin_mode;
	uint32_t pin_speed;
	uint32_t pin_pupd;
	uint32_t pin_optype;
	uint32_t pin_alt;

}gpio_config;


typedef struct     	// handle structure of GPIO pin
{
	gpio_reg *pGPIOx;  // ptr that hold the base address of peripheral
	gpio_config GPIO_Pinconfig;

}gpio_handle;


/* Driver API implementation */

 void gpio_init(gpio_handle *pGPIOhandle);									// gpio initialization
 void gpio_deinit(gpio_reg *pGPIOx);										// gpio deinitialization
 void gpio_clockcontrol(gpio_reg *pGPIOx, uint8_t EN_DIS); 					// gpio peripheral clock control
 uint8_t gpio_readinputpin(gpio_reg *pGPIOx, uint8_t pin_no);   			// gpio read from input pin
 uint16_t gpio_readinputport(gpio_reg *pGPIOx);								// gpio read from input port
 void gpio_writeoutputpin(gpio_reg *pGPIOx, uint8_t pin_no, uint16_t val); 	// gpio write to output pin
 void gpio_writeoutputport(gpio_reg *pGPIOx, uint16_t val);					// gpio write to output port
 void gpio_togglepin(gpio_reg *pGPIOx, uint8_t pin_no); 					// gpio toggle pin


 void gpio_irqconfig(uint8_t irq_no,uint8_t en_dis);						// gpio interrupt configuration
 void gpio_irqpriority(uint8_t irq_no,uint32_t priority);					// gpio interrupt priority configuration
 void gpio_irqhandle(uint8_t pin_no);										// gpio IRQ handling


 /*
  * @GPIO_PIN_NUMBERS
  * GPIO pin numbers
  */
 #define GPIO_PIN_0  				0
 #define GPIO_PIN_1  				1
 #define GPIO_PIN_2  				2
 #define GPIO_PIN_3  				3
 #define GPIO_PIN_4  				4
 #define GPIO_PIN_5  				5
 #define GPIO_PIN_6  				6
 #define GPIO_PIN_7  				7
 #define GPIO_PIN_8  				8
 #define GPIO_PIN_9  				9
 #define GPIO_PIN_10  				10
 #define GPIO_PIN_11 				11
 #define GPIO_PIN_12  				12
 #define GPIO_PIN_13 				13
 #define GPIO_PIN_14 				14
 #define GPIO_PIN_15 				15

 /*
  * @GPIO_PIN_MODES
  * GPIO pin possible modes
  */
 #define GPIO_MODE_IN 			0
 #define GPIO_MODE_OUT 			1
 #define GPIO_MODE_ALTFN 		2
 #define GPIO_MODE_ANALOG 		3

 #define GPIO_MODE_IT_FT    	4
 #define GPIO_MODE_IT_RT    	5
 #define GPIO_MODE_IT_RFT    	6


 /*
  * GPIO pin possible output types
  */
 #define GPIO_OP_TYPE_PP  	    0
 #define GPIO_OP_TYPE_OD   		1


 /*
  * @GPIO_PIN_SPEED
  * GPIO pin possible output speeds
  */
 #define GPIO_SPEED_LOW			0
 #define GPIO_SPEED_MEDIUM		1
 #define GPIO_SPEED_FAST		2
 #define GPOI_SPEED_HIGH		3


 /*
  * GPIO pin pull up AND pull down configuration macros
  */
 #define GPIO_NO_PUPD   		0
 #define GPIO_PIN_PU			1
 #define GPIO_PIN_PD			2


#endif /* INC_F401_GPIO_H_ */
