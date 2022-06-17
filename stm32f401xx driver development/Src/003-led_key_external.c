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

	// 	LED : A3 BUTTON :B5  IN OPEN DRAIN CONFIGURATION
	led.pGPIOx = GPIOA;
	led.GPIO_Pinconfig.pin_no = GPIO_PIN_3;
	led.GPIO_Pinconfig.pin_mode = GPIO_MODE_OUT;
	led.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;
	led.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	led.GPIO_Pinconfig.pin_pupd = GPIO_NO_PUPD;		//GPIO_NO_PUPD;

	button.pGPIOx = GPIOB;
	button.GPIO_Pinconfig.pin_no = GPIO_PIN_5;
	button.GPIO_Pinconfig.pin_mode = GPIO_MODE_IN;
	button.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;
	button.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	button.GPIO_Pinconfig.pin_pupd = GPIO_PIN_PU;	//GPIO_NO_PUPD; //


	gpio_clockcontrol(GPIOB, ENABLE);
	gpio_clockcontrol(GPIOA, ENABLE);
	gpio_init(&led);
	gpio_init(&button);

	while(1)
	{
		if(gpio_readinputpin(GPIOB, 5))
		{
			gpio_writeoutputpin(GPIOA, 3, 0);
			delay();
		}
		else
		{
			gpio_writeoutputpin(GPIOA, 3, 1);
			delay();
		}
	}

	return 0;


}
