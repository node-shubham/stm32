/*
 * 005-spi_tx.c
 * SPI slave ctrl programming and using printf debugging - semihosting
 *  Created on: 20-Dec-2021
 *      Author: iamshuvm
 */

#include "string.h"
#include "stm32f401xx.h"
#include "stdio.h"

//extern void initialise_monitor_handles(void);

//#define COMMAND_LED_CTRL 		0x50
#define COMMAND_SENSOR_READ 	0x51
//#define COMMAND_LED_READ 		0x52
//#define COMMAND_PRINT 		0x53
//#define COMMAND_ID_READ 		0x54

#define LED_ON 			1
#define LED_OFF 		0

#define ANALOG_PIN0 		0
#define ANALOG_PIN1 		1
#define ANALOG_PIN2 		2
#define ANALOG_PIN3 		3
#define ANALOG_PIN4 		4
#define ANALOG_PIN5 		5

#define LED_PIN 			9


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

uint8_t spi_verify_resp(uint8_t ack)
{
	if(ack == 0xf5)
		return 1;
	else
		return 0;
}

int main()
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

//	initialise_monitor_handles();

//	printf("Application is running\n");
	gpio_button();
	//char user_data[] = "hello newborn";
	spi2_gpioinit();
	spi2_init();
//	printf("SPI Init done\n");
	spi_ssoeconfig(SPI2, ENABLE);
	spi_peri_ctrl(SPI2,ENABLE);

	while(1)
	{


		spi_peri_ctrl(SPI2,ENABLE);
		while(gpio_readinputpin(GPIOB, 5));
	//	delay();
		delay();
		uint8_t ack_byte =0;
		uint8_t args[2];

	#ifdef COMMAND_LED_CTRL
		/*sending cmd COMMAND_LED_CTRL to arduino to get a response back*/
		uint8_t cmd = COMMAND_LED_CTRL;
	#endif
	#ifdef COMMAND_SENSOR_READ
		/*sending cmd COMMAND_SENSOR_READ to arduino to get a response back*/
		uint8_t cmd = COMMAND_SENSOR_READ;
	#endif
	#ifdef COMMAND_LED_READ
		/*sending cmd COMMAND_LED_READ to arduino to get a response back*/
		uint8_t cmd = COMMAND_LED_READ;
	#endif
	#ifdef COMMAND_PRINT
		/*sending cmd COMMAND_PRINT to arduino to get a response back*/
		uint8_t cmd = COMMAND_PRINT;
	#endif
	#ifdef COMMAND_ID_READ
		/*sending cmd COMMAND_ID_READ to arduino to get a response back*/
		uint8_t cmd = COMMAND_ID_READ;
	#endif

		spi_send(SPI2, &cmd,1);
		spi_receive(SPI2, &dummy_read, 1); //to clear off the RXNE
		spi_send(SPI2, &dummy_write,1);	//send dummy byte to fetch response from the slave
		spi_receive(SPI2, &ack_byte, 1);
		if(spi_verify_resp(ack_byte))
		{
			#ifdef COMMAND_LED_CTRL
					gpio_togglepin(GPIOA, 3);
					delay();
					gpio_togglepin(GPIOA, 3);
					delay();
					gpio_togglepin(GPIOA, 3);
					delay();
			#endif
			#ifdef COMMAND_LED_CTRL1		// Not working dont know why
					args[0]= LED_PIN;
					args[1]= LED_ON;
					spi_send(SPI2, args,2);
//					printf("CMD_LED_CTRL Executed\n");

					spi_receive(SPI2, args,2);
			#endif
			#ifdef COMMAND_SENSOR_READ
					args[0] = ANALOG_PIN0;

					//send arguments
					spi_send(SPI2,args,1); //sending one byte of

					//do dummy read to clear off the RXNE
					spi_receive(SPI2,&dummy_read,1);

					//insert some delay so that slave can ready with the data
					delay();

					//Send some dummy bits (1 byte) fetch the response from the slave
					spi_send(SPI2,&dummy_write,1);

					uint8_t analog_read;
					spi_receive(SPI2,&analog_read,1);
//					printf("COMMAND_SENSOR_READ %d\n",analog_read);
				#endif
		}

		//spi_send(SPI2, (uint8_t *)user_data,strlen(user_data));
		while(spi_flagstatus(SPI2, SPI_BSY_FLAG));
		spi_peri_ctrl(SPI2,DISABLE);
		gpio_writeoutputpin(GPIOA, 3, 0);

	}
	return 0;
}

