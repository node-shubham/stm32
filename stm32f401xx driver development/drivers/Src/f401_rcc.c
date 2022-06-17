/*
 * f401_rcc.c
 *
 *  Created on: 15-Jan-2022
 *      Author: iamshuvm
 */

#include "f401_rcc.h"

uint16_t ahb_prescaler[8] ={2,4,8,16,64,128,256,512};
uint16_t apb_prescaler[4] ={2,4,8,16};

uint32_t get_pll_clock_status()
{
	return 0;
}

uint32_t rcc_getpclk1_val(void)
{
	uint32_t pclk1,sysclk;
	uint8_t clksrc,temp,ahbp,apb1p;
	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0)
	{
		sysclk = 16000000;
	}else if(clksrc == 1)
	{
		sysclk = 8000000;
	}else if(clksrc == 2)
	{
		sysclk = get_pll_clock_status();
	}

	// for AHB prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp <8)
	{
		ahbp =1;
	}
	else
	{
		ahbp = ahb_prescaler[temp-8];
	}

	// for APB1 prescaler
	temp = (RCC->CFGR >> 10) & 0x7;
	if(temp <8)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = apb_prescaler[temp-4];
	}

	pclk1 = (sysclk/ahbp)/apb1p;

	return pclk1;
}

uint32_t rcc_getpclk2_val(void)
{
	uint32_t pclk2,sysclk;
	uint8_t clksrc,temp,ahbp,apb2p;
	clksrc = (RCC->CFGR >> 2) & 0x3;

	if(clksrc == 0)
	{
		sysclk = 16000000;
	}else if(clksrc == 1)
	{
		sysclk = 8000000;
	}else if(clksrc == 2)
	{
		sysclk = get_pll_clock_status();
	}

	// for AHB1 prescaler
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp <8)
	{
		ahbp =1;
	}
	else
	{
		ahbp = ahb_prescaler[temp-8];
	}

	// for APB2 prescaler
	temp = (RCC->CFGR >> 13) & 0x7;
	if(temp <8)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = apb_prescaler[temp-4];
	}

	pclk2 = (sysclk/ahbp)/apb2p;

	return pclk2;
}
