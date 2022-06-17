/*
 * f103_gpio.h
 *
 *  Created on: 22-Dec-2021
 *      Author: iamshuvm
 */

#ifndef INC_F103_GPIO_H_
#define INC_F103_GPIO_H_

#include "stm32f103xx.h"

typedef struct
{
	uint32_t pin_no;
	uint32_t pin_mode;
	uint32_t pin_otype;
	uint32_t pin_speed;

}gpio_config;

typedef struct
{
	gpio_reg *pGPIOx;
	gpio_config	gpio_pinconfig;

}gpio_handle;

#define GPIO_INPUT_MODE 		0
#define GPIO_OUTPUT_MODE   		1

#define GPIO_INPUT_ANALOG 		0
#define GPIO_INPUT_FLOATING		1
#define GPIO_INPUT_PU	 		2
#define GPIO_INPUT_PD	 		2

#define GPIO_OUTPUT_PP 			0
#define GPIO_OUTPUT_OD 			1
#define GPIO_AFIO_OP_PP 		2
#define GPIO_AFIO_OP_OD 		3

#define GPIO_SPEED_MEDIUM		1		// 10 MHz
#define GPIO_SPEED_LOW			2		// 2 MHz
#define GPIO_SPEED_HIGH			3		// 50 MHz

 /*
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
 #define GPIO_PIN_10  			10
 #define GPIO_PIN_11 				11
 #define GPIO_PIN_12  			12
 #define GPIO_PIN_13 				13
 #define GPIO_PIN_14 				14
 #define GPIO_PIN_15 				15


void gpio_clockctrl(gpio_reg *pGPIOx  ,uint8_t state);
void gpio_init(gpio_handle *pGPIO_handle);
void gpio_deinit(gpio_reg *pGPIOx);


void gpio_writepin(gpio_reg *pGPIOx, uint8_t pin_no, uint8_t state);
void gpio_writeport(gpio_reg *pGPIOx, uint8_t value);
uint8_t gpio_readpin(gpio_reg *pGPIOx, uint8_t pin_no);
uint32_t gpio_readport(gpio_reg *pGPIOx);
void gpio_togglepin(gpio_reg *pGPIOx, uint8_t pin_no); 					// gpio toggle pin



#endif /* INC_F103_GPIO_H_ */
