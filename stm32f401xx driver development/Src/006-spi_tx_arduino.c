/*
 * 005-spi_tx.c
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


/*  PB14 : SPI2 MISO
	PB15 : SPI2 MOSI
	PB13 : SPI2 SCLK
	PB12 : SPI2 NSS
*/
void spi2_gpioinit(void)
{
	gpio_handle spi_pin;

	spi_pin.pGPIOx = GPIOB;
	spi_pin.GPIO_Pinconfig.pin_mode = GPIO_MODE_ALTFN;
	spi_pin.GPIO_Pinconfig.pin_alt = 5;
	spi_pin.GPIO_Pinconfig.pin_optype = GPIO_OP_TYPE_PP;
	spi_pin.GPIO_Pinconfig.pin_pupd = GPIO_NO_PUPD;
	spi_pin.GPIO_Pinconfig.pin_speed = GPIO_SPEED_MEDIUM;

	//spi_pin.GPIO_Pinconfig.pin_no = 14;		// MISO
	//gpio_init(&spi_pin);
	spi_pin.GPIO_Pinconfig.pin_no = 15;		// MOSI
	gpio_init(&spi_pin);
	spi_pin.GPIO_Pinconfig.pin_no = 13;		// SCLK
	gpio_init(&spi_pin);
	//spi_pin.GPIO_Pinconfig.pin_no = 12;		// NSS
	//gpio_init(&spi_pin);
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

void spi2_init(void)
{
	spi_handle spi2_handle;
	spi2_handle.pSPIx = SPI2;
	spi2_handle.SPI_config.spi_devmode = SPI_MODE_MASTER;
	spi2_handle.SPI_config.spi_busconfig = SPI_BUS_FD;
	spi2_handle.SPI_config.spi_sclkspeed = SPI_SPEED_PSC8;
	spi2_handle.SPI_config.spi_dff = SPI_DFF_8BIT;
	spi2_handle.SPI_config.spi_cpol = SPI_CPOL_LOW;
	spi2_handle.SPI_config.spi_cpha = SPI_CPHA_LOW;
	spi2_handle.SPI_config.ssm = SPI_SSM_DI;			// HWM

	spi_init(&spi2_handle);
}

int main()
{
	gpio_button();
	char user_data[] = "hello newborn";
	spi2_gpioinit();
	spi2_init();
	spi_ssoeconfig(SPI2, ENABLE);
	spi_peri_ctrl(SPI2,ENABLE);

	while(1)
	{

		spi_peri_ctrl(SPI2,ENABLE);
		while(gpio_readinputpin(GPIOB, 5));
		gpio_writeoutputpin(GPIOA, 3, 1);
		//delay();
		//delay();
		uint8_t datalen = strlen(user_data);
		spi_send(SPI2, &datalen,1);
		spi_send(SPI2, (uint8_t *)user_data,strlen(user_data));
		while(spi_flagstatus(SPI2, SPI_BSY_FLAG));
		spi_peri_ctrl(SPI2,DISABLE);
		gpio_writeoutputpin(GPIOA, 3, 0);

	}
	return 0;
}



