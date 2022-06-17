/*

 *
 *  Created on: 30-Dec-2021
 *      Author: iamshuvm
 */

#include "stdio.h"
#include "string.h"
#include "stm32f401xx.h"


#define SLAVE_ADDR		0x69  //0x69 for ack failure
#define MY_ADDR 	SLAVE_ADDR

uint8_t rx_cplt = RESET;
void delay(void)
{
	for(uint32_t i=0; i<500000/2;i++);		//appx 200ms delay
}

i2c_handle i2c1_handle;

uint8_t data[32] ="stm32 slave mode test" ;
uint8_t len =0;

uint8_t cmd_len = 0x51;
uint8_t cmd_data = 0x52;

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
	i2c_irqconfig(IRQ_NO_I2C1_EV,ENABLE);
	i2c_irqconfig(IRQ_NO_I2C1_ER,ENABLE);

	i2c_slave_enordi_callback(I2C1,ENABLE);
	i2c_peri_ctrl(I2C1, ENABLE);

	i2c_manage_ack(I2C1, ENABLE);
	while(1);
	return 0;

}

void I2C1_EV_IRQHandler(void)           			/* I2C1 event interrupt   */
{
	i2c_ev_irqhnadling(&i2c1_handle);
}

void I2C1_ER_IRQHandler(void)           			/* I2C1 error interrupt */
{
	i2c_err_irqhnadling(&i2c1_handle);
}


void i2c_event_callback(i2c_handle *pI2Chandle,uint8_t app_event)
{
	static uint8_t cmd_code =0;
	static uint8_t cnt =0;
	if(app_event == I2C_EV_DATA_REQ)
	{
		if(cmd_code == 0x51)
			i2c_slave_senddata(pI2Chandle->pI2Cx, strlen((char *)data));
		else if(cmd_code == 0x52)
			i2c_slave_senddata(pI2Chandle->pI2Cx, data[cnt++]);
	}
	else if(app_event == I2C_EV_DATA_RCV)
	{
		cmd_code = i2c_slave_receivedata(pI2Chandle->pI2Cx);
	}
	else if(app_event == I2C_ERROR_AF)
	{
		cmd_code = 0xff;
		cnt =0;
	}
	else if(app_event == I2C_EV_STOP)
	{


	}
}
