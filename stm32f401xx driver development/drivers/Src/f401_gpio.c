/*
 * f401_gpio.c
 *
 *  Created on: 19-Dec-2021
 *      Author: shubham saha ray
 */

#include "f401_gpio.h"


/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

 void gpio_clockcontrol(gpio_reg *pGPIOx, uint8_t state)
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
		 else if(pGPIOx == GPIOH)
		 {
			 GPIOH_PCLK_EN();
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
		 else if(pGPIOx == GPIOH)
		 {
			 GPIOH_PCLK_DI();
		 }
	 }
 }

 /*********************************************************************
  * @fn      		  - GPIO_Init
  *
  * @brief             - gpio initialization
  *
  * @param[in]         - base address of the gpio peripheral
  * @param[in]         -
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  none
  */

 void gpio_init(gpio_handle *pGPIOhandle)
 {
	 uint32_t temp = 0;
	 uint32_t temp1 = 0;
	 uint32_t temp2 = 0;

	 // enable the peripheral clock
	 gpio_clockcontrol(pGPIOhandle->pGPIOx, ENABLE);

	 if(pGPIOhandle->GPIO_Pinconfig.pin_mode <= GPIO_MODE_ANALOG )
	 {

		// configure the mode
		temp = (pGPIOhandle->GPIO_Pinconfig.pin_mode << (2* pGPIOhandle->GPIO_Pinconfig.pin_no));
		pGPIOhandle->pGPIOx->MODER &= ~(0x3 << pGPIOhandle->GPIO_Pinconfig.pin_no );
		pGPIOhandle->pGPIOx->MODER |= temp;

	 }
	 else
	 {
		 	 // interrupt mode
		 if(pGPIOhandle->GPIO_Pinconfig.pin_mode == GPIO_MODE_IT_FT)
		 {
			 // configure the FTSR
			 EXTI->FTSR |= (1<<pGPIOhandle->GPIO_Pinconfig.pin_no);
			 EXTI->RTSR &= ~(1<<pGPIOhandle->GPIO_Pinconfig.pin_no);

		 }
		 else if(pGPIOhandle->GPIO_Pinconfig.pin_mode == GPIO_MODE_IT_RT)
		 {
			 // configure the RTSR
			 EXTI->RTSR |= (1<<pGPIOhandle->GPIO_Pinconfig.pin_no);
			 EXTI->FTSR &= ~(1<<pGPIOhandle->GPIO_Pinconfig.pin_no);

		 } else if(pGPIOhandle->GPIO_Pinconfig.pin_mode == GPIO_MODE_IT_RFT )
		 {
			 // configure the FTSR and RTSR
			 EXTI->RTSR |= (1<<pGPIOhandle->GPIO_Pinconfig.pin_no);
			 EXTI->FTSR |= (1<<pGPIOhandle->GPIO_Pinconfig.pin_no);

		 }

		 //configure the GPIO port selection in SYSCFG_EXTICR
		 temp1 = pGPIOhandle->GPIO_Pinconfig.pin_no /4;
		 temp2 = pGPIOhandle->GPIO_Pinconfig.pin_no %4;
		 uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOhandle->pGPIOx);
		 SYSCFG_PCLK_EN();
		 SYSCFG->EXTICR[temp1] |= portcode << (4* temp2 );

		 // enable the EXTI interrupt delivery using IMR
		 EXTI->IMR |= (1<<pGPIOhandle->GPIO_Pinconfig.pin_no);
	 }
	 // configure the speed
	 temp =0;
	 temp = (pGPIOhandle->GPIO_Pinconfig.pin_speed << (2* pGPIOhandle->GPIO_Pinconfig.pin_no));
	 pGPIOhandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOhandle->GPIO_Pinconfig.pin_no );
	 pGPIOhandle->pGPIOx->OSPEEDR |= temp;

	 // configure the PUPD
	 temp =0;
	 temp = (pGPIOhandle->GPIO_Pinconfig.pin_pupd << (2* pGPIOhandle->GPIO_Pinconfig.pin_no));
	 pGPIOhandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOhandle->GPIO_Pinconfig.pin_no );
	 pGPIOhandle->pGPIOx->PUPDR |= temp;

	 // configure the OType
	 temp =0;
	 temp = (pGPIOhandle->GPIO_Pinconfig.pin_optype <<  pGPIOhandle->GPIO_Pinconfig.pin_no);
	 pGPIOhandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOhandle->GPIO_Pinconfig.pin_no );
	 pGPIOhandle->pGPIOx->OTYPER |= temp;

	 // configure the ALT function

	 if(pGPIOhandle->GPIO_Pinconfig.pin_mode == GPIO_MODE_ALTFN )
	 {
		  temp1 =0;
		  temp2 =0;

		  temp1 = (pGPIOhandle->GPIO_Pinconfig.pin_no)/8;
		  temp2 = (pGPIOhandle->GPIO_Pinconfig.pin_no)%8;

		  pGPIOhandle->pGPIOx->AFR[temp1] &= ~(0xF << (4* temp2)) ;
		  pGPIOhandle->pGPIOx->AFR[temp1] |=(pGPIOhandle->GPIO_Pinconfig.pin_alt<< (4* temp2)) ;
	 }
}

 /*********************************************************************
  * @fn      		  - // gpio deinitialization
  *
  * @brief             - This function enables or disables peripheral clock for the given GPIO port
  *
  * @param[in]         - base address of the gpio peripheral
  * @param[in]         - ENABLE or DISABLE macros
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  none
  */




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
 /*********************************************************************
  * @fn      		  - GPIO_PeriClockControl
  *
  * @brief             - gpio read from input pin
  *
  * @param[in]         - base address of the gpio peripheral
  * @param[in]         - ENABLE or DISABLE macros
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  none
  */

 uint8_t gpio_readinputpin(gpio_reg *pGPIOx, uint8_t pin_no)   						//
 {
	 uint8_t value;
	 value = (uint8_t) ((pGPIOx->IDR >> pin_no)& 0x00000001 );
	 return value;
 }


 /*********************************************************************
  * @fn      		  - GPIO_PeriClockControl
  *
  * @brief             - // gpio read from input port
  *
  * @param[in]         - base address of the gpio peripheral
  * @param[in]         - ENABLE or DISABLE macros
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  none
  */

 uint16_t gpio_readinputport(gpio_reg *pGPIOx)
 {

	 uint16_t value;
	 value = (uint16_t) ( pGPIOx->IDR );
	 return value;
 }
 /*********************************************************************
  * @fn      		  - gpio_writeoutputpin
  *
  * @brief             -  gpio write to output pin
  *
  * @param[in]         - base address of the gpio peripheral
  * @param[in]         - pin number
  * @param[in]         - value
  *
  * @return            -  none
  *
  * @Note              -  none
  */

 void gpio_writeoutputpin(gpio_reg *pGPIOx, uint8_t pin_no, uint16_t value)
 {
	 if(value == GPIO_PIN_SET)
	 {
		 pGPIOx->ODR |= (1<<pin_no);
	 }
	 else
	 {
		 pGPIOx->ODR &= ~(1<<pin_no);
	 }

 }

 /*********************************************************************
  * @fn      		  -	gpio_writeoutputport
  *
  * @brief             -  gpio write to output port
  *
  * @param[in]         -
  * @param[in]         -
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  none
  */


 void gpio_writeoutputport(gpio_reg *pGPIOx, uint16_t value)
 {
	 pGPIOx->ODR = value;
 }


 /*********************************************************************
  * @fn      		  -
  *
  * @brief             -	 gpio toggle pin
  *
  * @param[in]         -
  * @param[in]         -
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  none
  */
 void gpio_togglepin(gpio_reg *pGPIOx, uint8_t pin_no)
 {
	 pGPIOx->ODR ^= (1<< pin_no);
 }

 /*********************************************************************
  * @fn      		  -
  *
  * @brief             -	gpio interrupt configuration
  *
  * @param[in]         -
  * @param[in]         -
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  PLEASE refer CORTEX M4 devices generic user manual from ARM for ISER and ICER registers
  */
 void gpio_irqconfig(uint8_t irq_no,uint8_t state)
 {
	 if(state == ENABLE)
	 {
		 if(irq_no <= 31)
		 {
			  *NVIC_ISER0 |= (1<< irq_no); 			// ISER0
		 }else if(irq_no >= 31 && irq_no <64)
		 {
			 *NVIC_ISER1 |= (1<< (irq_no %32)); 	// ISER1
		 }
		 else if(irq_no >= 64 && irq_no <96)
		 {
			 *NVIC_ISER2 |= (1<< (irq_no %64));		//ISER2
		 }

	 }
	 else
	 {
		 if(irq_no <= 31)
		 {
			 *NVIC_ICER0 |= (1<< irq_no);			//ICER0
		 }else if(irq_no >= 31 && irq_no <64)
		 {
			 *NVIC_ICER1 |= (1<< (irq_no %32));		//ICER1
		 }
		 else if(irq_no >= 64 && irq_no <96)
		 {
			 *NVIC_ICER2 |= (1<< (irq_no %64));		//ICER2
		 }


	 }
 }

 /*********************************************************************
  * @fn      		  -		gpio_irqpriority
  *
  * @brief             -	 gpio IRQ priority control
  *
  * @param[in]         -
  * @param[in]         -
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  none
  */

 void gpio_irqpriority(uint8_t irq_no,uint32_t priority)
 {

	 uint8_t iprx = irq_no/4;
	 uint8_t iprx_section = irq_no%4;
	 uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IPR_BITS);
	 *(NVIC_IPR_BASEADDR + iprx) |= (priority << shift_amount);

 }

 /*********************************************************************
  * @fn      		  -	   gpio_irqhandle
  *
  * @brief             -	gpio IRQ handling
  *
  * @param[in]         -	GPIO pin number
  * @param[in]         -
  * @param[in]         -
  *
  * @return            -  none
  *
  * @Note              -  none
  */
 void gpio_irqhandle(uint8_t pin_no)
 {
	 if(EXTI->PR & (1<<pin_no))
		 EXTI->PR |= (1<<pin_no);
 }
