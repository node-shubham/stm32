/*
 * f401_rcc.h
 *
 *  Created on: 15-Jan-2022
 *      Author: iamshuvm
 */

#ifndef INC_F401_RCC_H_
#define INC_F401_RCC_H_

#include "stm32f401xx.h"

uint32_t get_pll_clock_status(void);
uint32_t rcc_getpclk1_val(void);
uint32_t rcc_getpclk2_val(void);

#endif /* INC_F401_RCC_H_ */
