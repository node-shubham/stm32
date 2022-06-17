/*
 * 005-spi_tx.c
 * SPI slave ctrl programming and using printf debugging - semihosting
 *  Created on: 20-Dec-2021
 *      Author: iamshuvm
 */

#include "string.h"
#include "stm32f401xx.h"
#include "stdio.h"

extern void initialise_monitor_handles(void);

#define COMMAND_LED_CTRL 		0x50
#define COMMAND_SENSOR_READ 	0x51
#define COMMAND_LED_READ 		0x52
#define COMMAND_PRINT 			0x53
#define COMMAND_ID_READ 		0x54

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



int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	initialise_monitor_handles();

	printf("Application is running\n");

	gpio_button();
	//char user_data[] = "hello newborn";
	spi2_gpioinit();
	spi2_init();
//	printf("SPI Init done\n");
	spi_ssoeconfig(SPI2, ENABLE);




	printf("SPI Init. done\n");

	/*
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/


	while(1)
	{
		//wait till button is pressed
		while( gpio_readinputpin(GPIOB,GPIO_PIN_5) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral
		spi_peri_ctrl(SPI2,ENABLE);

	    //1. CMD_LED_CTRL  	<pin no(1)>     <value(1)>

		uint8_t commandcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		//send command
		spi_send(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		spi_receive(SPI2,&dummy_read,1);


		//Send some dummy bits (1 byte) fetch the response from the slave
		spi_send(SPI2,&dummy_write,1);

		//read the ack byte received
		spi_receive(SPI2,&ackbyte,1);

		if( spi_verify_resp(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//send arguments
			spi_send(SPI2,args,2);
			// dummy read
			spi_receive(SPI2,args,2);
			printf("COMMAND_LED_CTRL Executed\n");
		}
		//end of COMMAND_LED_CTRL




		//2. CMD_SENOSR_READ   <analog pin number(1) >

		//wait till button is pressed
		while( gpio_readinputpin(GPIOB,GPIO_PIN_5) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		spi_send(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		spi_receive(SPI2,&dummy_read,1);


		//Send some dummy byte to fetch the response from the slave
		spi_send(SPI2,&dummy_write,1);

		//read the ack byte received
		spi_receive(SPI2,&ackbyte,1);

		if( spi_verify_resp(ackbyte))
		{
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
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while( gpio_readinputpin(GPIOB,GPIO_PIN_5) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		spi_send(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		spi_receive(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		spi_send(SPI2,&dummy_write,1);

		//read the ack byte received
		spi_receive(SPI2,&ackbyte,1);

		if( spi_verify_resp(ackbyte))
		{
			args[0] = LED_PIN;

			//send arguments
			spi_send(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			spi_receive(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			spi_send(SPI2,&dummy_write,1);

			uint8_t led_status;
			spi_receive(SPI2,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while( gpio_readinputpin(GPIOB,GPIO_PIN_5) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		//send command
		spi_send(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		spi_receive(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		spi_send(SPI2,&dummy_write,1);

		//read the ack byte received
		spi_receive(SPI2,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( spi_verify_resp(ackbyte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			spi_send(SPI2,args,1); //sending length

			//do dummy read to clear off the RXNE
			spi_receive(SPI2,&dummy_read,1);

			delay();

			//send message
			for(int i = 0 ; i < args[0] ; i++){
				spi_send(SPI2,&message[i],1);
				spi_receive(SPI2,&dummy_read,1);
			}

			printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while( gpio_readinputpin(GPIOB,GPIO_PIN_5) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		spi_send(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		spi_receive(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		spi_send(SPI2,&dummy_write,1);

		//read the ack byte received
		spi_receive(SPI2,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( spi_verify_resp(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				spi_send(SPI2,&dummy_write,1);
				spi_receive(SPI2,&id[i],1);
			}

			id[10] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}

		//lets confirm SPI is not busy
		while( spi_flagstatus(SPI2,SPI_BSY_FLAG) );

		//Disable the SPI2 peripheral
		spi_peri_ctrl(SPI2,DISABLE);

		printf("SPI Communication Closed\n");
	}

	return 0;

}

