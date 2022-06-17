/*

 *
 *  Created on: 30-Dec-2021
 *      Author: iamshuvm
 */

#include "string.h"
#include "stm32f401xx.h"


void delay(void)
{
	for(uint32_t i=0; i<500000/2;i++);		//appx 200ms delay
}

usart_handle usart1_handle;
char data[] = "hey, message via USART protocol - shubham\n\r";

/*  PA9 : usart1_tx ---A5 ARDUINO
	PA10 : usart1_rx ---A4 ARDUINO
*/

void usart1_gpioinit(void)
{
	gpio_handle usart_pin;

	usart_pin.pGPIOx = GPIOA;
	usart_pin.GPIO_Pinconfig.pin_mode = GPIO_MODE_ALTFN;
	usart_pin.GPIO_Pinconfig.pin_alt = 7;
	usart_pin.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	usart_pin.GPIO_Pinconfig.pin_pupd = GPIO_PIN_PU;
	usart_pin.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;


	usart_pin.GPIO_Pinconfig.pin_no = 9;		// TX
	gpio_init(&usart_pin);
	usart_pin.GPIO_Pinconfig.pin_no = 10;		// RX
	gpio_init(&usart_pin);

}

void gpio_button(void)
{

	gpio_handle led , button;

	// 	LED : A3 BUTTON :B5
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

	gpio_init(&led);
	gpio_init(&button);
}

void usart1_init(void)
{


	usart1_handle.pUSARTx = USART1;
	usart1_handle.USART_config.usart_baud = USART_BAUD_9600;
	usart1_handle.USART_config.usart_mode = USART_MODE_TX;
	usart1_handle.USART_config.usart_wordlen = USART_WORDLEN_8BITS;
	usart1_handle.USART_config.usart_parity = USART_PARITY_DISABLE;
	usart1_handle.USART_config.usart_stopbit = USART_STOPBITS_1;
	usart1_handle.USART_config.usart_hwflow = USART_HW_FLOW_CTRL_NONE;

	usart_init(&usart1_handle);
}

int main()
{
	usart1_gpioinit();
	gpio_button();
	usart1_init();
	usart_peri_ctrl(USART1, ENABLE);

	while(1)
	{
		while(gpio_readinputpin(GPIOB, 5));
		delay();
		gpio_writeoutputpin(GPIOA, 3, SET);
		usart_senddata(&usart1_handle, (uint8_t *)data, strlen(data));
		gpio_writeoutputpin(GPIOA, 3, RESET);
	}
	return 0;
}



