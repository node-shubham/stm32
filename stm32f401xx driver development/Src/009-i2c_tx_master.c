/*

 *
 *  Created on: 30-Dec-2021
 *      Author: iamshuvm
 */

#include "string.h"
#include "stm32f401xx.h"

#define MY_ADDR 		0x61
#define SLAVE_ADDR		0x68

void delay(void)
{
	for(uint32_t i=0; i<500000/2;i++);		//appx 200ms delay
}

i2c_handle i2c1_handle;
uint8_t data[] = "hey, message via I2C protocol";

/*  PB6 : I2C1 SCL---A5 ARDUINO
	PB7 : I2C1 SDA---A4 ARDUINO
*/

void i2c1_gpioinit(void)
{
	gpio_handle i2c_pin;

	i2c_pin.pGPIOx = GPIOB;
	i2c_pin.GPIO_Pinconfig.pin_mode = GPIO_MODE_ALTFN;
	i2c_pin.GPIO_Pinconfig.pin_alt = 4;
	i2c_pin.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_OD;
	i2c_pin.GPIO_Pinconfig.pin_pupd = GPIO_PIN_PU;
	i2c_pin.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;


	i2c_pin.GPIO_Pinconfig.pin_no = 6;		// SCL
	gpio_init(&i2c_pin);
	i2c_pin.GPIO_Pinconfig.pin_no = 7;		// SDA
	gpio_init(&i2c_pin);

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

void i2c1_init(void)
{


	i2c1_handle.pI2Cx = I2C1;
	i2c1_handle.I2C_config.ack_ctrl = I2C_ACK_ENABLE;
	i2c1_handle.I2C_config.device_addr = MY_ADDR;
	i2c1_handle.I2C_config.scl_speed = I2C_SCl_SPEED_SM;
	i2c1_handle.I2C_config.fm_duty_cycle = I2C_FM_DUTY2;


	i2c_init(&i2c1_handle);
}

int main()
{
	i2c1_gpioinit();
	gpio_button();
	i2c1_init();
	i2c_peri_ctrl(I2C1, ENABLE);

	while(1)
	{
		while(gpio_readinputpin(GPIOB, 5));
		delay();
		gpio_writeoutputpin(GPIOA, 3, SET);
		i2c_senddata(&i2c1_handle, data, strlen((char *)data), SLAVE_ADDR);
		gpio_writeoutputpin(GPIOA, 3, RESET);
	}
	return 0;
}



