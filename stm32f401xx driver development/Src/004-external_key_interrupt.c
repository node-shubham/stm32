/*
 * 001-led_toggle.c
 *
 *  Created on: 20-Dec-2021
 *      Author: iamshuvm
 */

#include "string.h"
#include "stm32f401xx.h"

void delay(void)
{
	for(uint32_t i=0; i<500000/2;i++);		//appx 200ms delay
}

int main()
{
	gpio_handle led , button;
	memset(&led, 0,sizeof(led));
	memset(&button, 0,sizeof(button)); 	// initialize each every member of structure to 0

	// 	LED : A3 BUTTON :B5   TOGGLE PIN WHEN BUTTON IS PRESSED (WITH INTERRUPT falling edge)
	led.pGPIOx = GPIOA;
	led.GPIO_Pinconfig.pin_no = GPIO_PIN_3;
	led.GPIO_Pinconfig.pin_mode = GPIO_MODE_OUT;
	led.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;
	led.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	led.GPIO_Pinconfig.pin_pupd = GPIO_NO_PUPD;

	gpio_clockcontrol(GPIOA, ENABLE);
	gpio_init(&led);

	button.pGPIOx = GPIOB;
	button.GPIO_Pinconfig.pin_no = GPIO_PIN_5;
	button.GPIO_Pinconfig.pin_mode = GPIO_MODE_IT_FT;			//IT FALLING EDGE
	button.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;
	button.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	button.GPIO_Pinconfig.pin_pupd = GPIO_PIN_PU;

	gpio_clockcontrol(GPIOB, ENABLE);
	gpio_init(&button);

	//	IRQ configuration
	gpio_irqconfig(IRQ_NO_EXTI9_5, ENABLE);
	gpio_irqpriority(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);

	while(1);

	// IMPLEMENT ISR : GOTO STARTUP FILE

	return 0;
}


void EXTI9_5_IRQHandler(void)
{
	delay();
	gpio_irqhandle(GPIO_PIN_5);			// clear pending bit
	gpio_togglepin(GPIOA, GPIO_PIN_3);
}
