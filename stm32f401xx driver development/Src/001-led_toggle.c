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
	gpio_handle led;

	///* 	USING PUSH PULL CONFIGURATION
	led.pGPIOx = GPIOC;
	led.GPIO_Pinconfig.pin_no = GPIO_PIN_13;
	led.GPIO_Pinconfig.pin_mode = GPIO_MODE_OUT;
	led.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;
	led.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	led.GPIO_Pinconfig.pin_pupd = GPIO_NO_PUPD;
	//*/
	/* 	USING OPEN DRAIN CONFIGURATION */

	/*
	led.pGPIOx = GPIOC;
	led.GPIO_Pinconfig.pin_no = GPIO_PIN_13;
	led.GPIO_Pinconfig.pin_mode = GPIO_MODE_OUT;
	led.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;
	led.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_OD;
	led.GPIO_Pinconfig.pin_pupd = GPIO_PIN_PD;
	*/

	gpio_clockcontrol(GPIOC, ENABLE);
	gpio_init(&led);


	while(1)
	{
		gpio_togglepin(GPIOC, 13);
		delay();

	}

	return 0;


}
