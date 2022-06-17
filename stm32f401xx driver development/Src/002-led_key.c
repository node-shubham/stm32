/*
 * 001-led_toggle.c
 *
 *  Created on: 20-Dec-2021
 *      Author: iamshuvm
 */

#include "stm32f401xx.h"

void delay(void)
{
	for(uint32_t i=0; i<500000;i++);
}

int main()
{
	gpio_handle led , button;

	// 	USING PUSH PULL CONFIGURATION
	led.pGPIOx = GPIOC;
	led.GPIO_Pinconfig.pin_no = GPIO_PIN_13;
	led.GPIO_Pinconfig.pin_mode = GPIO_MODE_OUT;
	led.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;
	led.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	led.GPIO_Pinconfig.pin_pupd = GPIO_NO_PUPD;

	button.pGPIOx = GPIOA;
	button.GPIO_Pinconfig.pin_no = GPIO_PIN_0;
	button.GPIO_Pinconfig.pin_mode = GPIO_MODE_IN;
	button.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;
	button.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	button.GPIO_Pinconfig.pin_pupd = GPIO_PIN_PU;//GPIO_NO_PUPD; // MCU key need to be pulled up


	gpio_clockcontrol(GPIOC, ENABLE);
	gpio_clockcontrol(GPIOA, ENABLE);
	gpio_init(&led);
	gpio_init(&button);

	while(1)
	{
		if(gpio_readinputpin(GPIOA, 0))
		{
			gpio_writeoutputpin(GPIOC, 13, 1);
			//delay();
		}
		else
		{
			gpio_writeoutputpin(GPIOC, 13, 0);
			//delay();
		}
	}

	return 0;


}
