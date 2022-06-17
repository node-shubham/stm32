/*
 * f103_gpio.c
 *
 *  Created on: 22-Dec-2021
 *      Author: shubham
 */

#include "f103_gpio.h"


void gpio_clockctrl(gpio_reg *pGPIOx  ,uint8_t state)
{
	if(state == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
	}
}
void gpio_init(gpio_handle *pGPIO_handle)
{
	uint32_t temp =0;
	//temp=pGPIO_handle->gpio_pinconfig.pin_mode <<(4*pGPIO_handle->gpio_pinconfig.pin_no);
//	pGPIO_handle->gpio_pinconfig.pin_no =13;
	 /* enable the peripheral clock */
		gpio_clockctrl(pGPIO_handle->pGPIOx , ENABLE);

	
	if(pGPIO_handle->gpio_pinconfig.pin_no >=8)
	{
		// configure the mode     (input mode or output mode)
		pGPIO_handle->pGPIOx->CRH &=	~(0xf<<(4*(pGPIO_handle->gpio_pinconfig.pin_no -8)));
		temp |= (pGPIO_handle->gpio_pinconfig.pin_mode <<(4*(pGPIO_handle->gpio_pinconfig.pin_no -8)));
	
		// configure the mode type for CRH
		temp |= pGPIO_handle->gpio_pinconfig.pin_otype <<((2<<(4*(pGPIO_handle->gpio_pinconfig.pin_no -8))));
	}
	else
	{	
		// configure the mode     (input mode or output mode)
		temp |= (pGPIO_handle->gpio_pinconfig.pin_mode <<(4*pGPIO_handle->gpio_pinconfig.pin_no));
		pGPIO_handle->pGPIOx->CRH &=	~(0xf <<(4*pGPIO_handle->gpio_pinconfig.pin_no));
		
		// configure the mode type for CRL
		temp |= pGPIO_handle->gpio_pinconfig.pin_otype <<((2<<(4*pGPIO_handle->gpio_pinconfig.pin_no)));
	}
	
	// configure the speed
	if(pGPIO_handle->gpio_pinconfig.pin_speed > GPIO_INPUT_MODE)
		temp |= pGPIO_handle->gpio_pinconfig.pin_speed <<(4*pGPIO_handle->gpio_pinconfig.pin_no);


	if(pGPIO_handle->gpio_pinconfig.pin_no >=8)
		pGPIO_handle->pGPIOx->CRH |= temp;
	else
		pGPIO_handle->pGPIOx->CRL |= temp;

}
/*
void gpio_deinit(gpio_reg *pGPIOx)
{
	 if(pGPIOx == GPIOA)
		 GPIOA_REG_RESET();

	 else if(pGPIOx == GPIOB)
		 GPIOB_REG_RESET();

	 else if(pGPIOx == GPIOC)
		 GPIOC_REG_RESET();

	 else if(pGPIOx == GPIOD)
		 GPIOD_REG_RESET();

	 else if(pGPIOx == GPIOE)
		 GPIOE_REG_RESET();

	 else if(pGPIOx == GPIOH)
		 GPIOH_REG_RESET();
}
*/

void gpio_writepin(gpio_reg *pGPIOx, uint8_t pin_no, uint8_t state)
{
	if (state == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1<< pin_no);
	}
	else
	{
		pGPIOx->ODR |= (1<< pin_no);
	}
}
void gpio_writeport(gpio_reg *pGPIOx, uint8_t value)
{
	pGPIOx->ODR = value;
}

uint8_t gpio_readpin(gpio_reg *pGPIOx, uint8_t pin_no)
{
	uint8_t val=0;
	val = (0x00000001&((pGPIOx->IDR)>>pin_no));
	return val;
}
uint32_t gpio_readport(gpio_reg *pGPIOx)
{
	uint32_t val=0;
	val = pGPIOx->IDR;
	return val;

}
void gpio_togglepin(gpio_reg *pGPIOx, uint8_t pin_no) 					// gpio toggle pin
{
	pGPIOx->ODR ^= (1<<pin_no);
}
