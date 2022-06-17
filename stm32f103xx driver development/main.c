
#include "stm32f103xx.h"
#include "f103_gpio.h"

void delay(uint32_t num);

void delay(uint32_t num)
{	
	for(uint32_t j=0;j<num;j++)
		for(uint32_t i=0;i<2000;i++);
}

int main()
{
	//GPIOC_PCLK_EN();
	//gpio_clockctrl(GPIOC , ENABLE);
		gpio_handle abc;
		abc.pGPIOx = GPIOC;
	abc.gpio_pinconfig.pin_no = GPIO_PIN_13;
		abc.gpio_pinconfig.pin_mode = GPIO_OUTPUT_MODE;
		abc.gpio_pinconfig.pin_otype = GPIO_OUTPUT_PP;
	abc.gpio_pinconfig.pin_speed= GPIO_SPEED_LOW;
	gpio_init(&abc);
	while(1)
	{
		gpio_togglepin(GPIOC, 13);
		delay(500);	
	}	
}	

