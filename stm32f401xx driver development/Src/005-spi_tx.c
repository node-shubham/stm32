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

	spi_pin.GPIO_Pinconfig.pin_no = 14;		// MISO
	gpio_init(&spi_pin);
	spi_pin.GPIO_Pinconfig.pin_no = 15;		// MOSI
	gpio_init(&spi_pin);
	spi_pin.GPIO_Pinconfig.pin_no = 13;		// SCLK
	gpio_init(&spi_pin);
	spi_pin.GPIO_Pinconfig.pin_no = 12;		// NSS
	gpio_init(&spi_pin);
}

void spi2_init(void)
{
	spi_handle spi2_handle;
	spi2_handle.pSPIx = SPI2;
	spi2_handle.SPI_config.spi_devmode = SPI_MODE_MASTER;
	spi2_handle.SPI_config.spi_busconfig = SPI_BUS_FD;
	spi2_handle.SPI_config.spi_sclkspeed = SPI_SPEED_PSC2;
	spi2_handle.SPI_config.spi_dff = SPI_DFF_8BIT;
	spi2_handle.SPI_config.spi_cpol = SPI_CPOL_LOW;
	spi2_handle.SPI_config.spi_cpha = SPI_CPHA_LOW;
	spi2_handle.SPI_config.ssm = SPI_SSM_EN;

	spi_init(&spi2_handle);
}

int main()
{

	char user_data[] = "hello shubham";
	spi2_gpioinit();
	spi2_init();
	spi_peri_ctrl(SPI2,ENABLE);	// add ssi function
	spi_send(SPI2, (uint8_t *)user_data,strlen(user_data));
	while(spi_flagstatus(SPI2, SPI_BSY_FLAG));
	spi_peri_ctrl(SPI2,DISABLE);
	//while(1);
	return 0;
}


