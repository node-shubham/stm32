/*
 * f401_usart.c
 *
 *  Created on: 12-Jan-2022
 *      Author: iamshuvm
 */

#include "stm32f401xx.h"


void usart_clockctrl(usart_reg *pUSARTx, uint8_t state)
{
	if(state == ENABLE)
	 {
		 if(pUSARTx == USART1)
		 {
			 USART1_PCLK_EN();
		 }
		 else if(pUSARTx == USART2)
		 {
			 USART2_PCLK_EN();
		 }
		 else if(pUSARTx == USART6)
		 {
			 USART6_PCLK_EN();
		 }

	 }
	else
	 {
		 if(pUSARTx == USART1)
		 {
			 USART1_PCLK_DI();
		 }
		 else if(pUSARTx == USART2)
		 {
			 USART2_PCLK_DI();
		 }
		 else if(pUSARTx == USART6)
		 {
			 USART6_PCLK_DI();
		 }
	 }


}

void usart_baudrate(usart_reg *pUSARTx, uint32_t baud_rate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t m_part,f_part;

  uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = rcc_getpclk2_val();
  }else
  {
	   PCLKx = rcc_getpclk1_val();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLKx) / (2 *baud_rate));
  }else
  {
	   //over sampling by 16
	  usartdiv = ((25 * PCLKx) / (4 *baud_rate));
  }

  //Calculate the Mantissa part
  m_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= m_part << 4;

  //Extract the fraction part
  f_part = (usartdiv - (m_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  f_part = ((( f_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   f_part = ((( f_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= f_part;

  //copy the value of tempreg in to BRR register
  pUSARTx->BRR |= tempreg;
}
void usart_init(usart_handle *pUSARThandle)
{

	usart_clockctrl(pUSARThandle->pUSARTx, ENABLE);

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
	usart_clockctrl(pUSARThandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARThandle->USART_config.usart_mode == USART_MODE_TX)
	{
		//Implement the code to enable the Receiver bit field
		tempreg|= (1 << USART_CR1_TE);
	}else if (pUSARThandle->USART_config.usart_mode == USART_MODE_RX)
	{
		//Implement the code to enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_RE );

	}else if (pUSARThandle->USART_config.usart_mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields
		tempreg |= ( ( 1 << USART_CR1_TE) | ( 1 << USART_CR1_RE) );
	}

	//Implement the code to configure the Word length configuration item
	tempreg |= pUSARThandle->USART_config.usart_wordlen << USART_CR1_M ;


	//Configuration of parity control bit fields
	if ( pUSARThandle->USART_config.usart_parity == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity
		//Not required because by default EVEN parity will be selected once you enable the parity control

	}else if (pUSARThandle->USART_config.usart_parity == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable ODD parity
		tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARThandle->pUSARTx->CR1 |= tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission
	tempreg |= pUSARThandle->USART_config.usart_stopbit << USART_CR2_STOP ;

	//Program the CR2 register
	pUSARThandle->pUSARTx->CR2 |= tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARThandle->USART_config.usart_hwflow == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARThandle->USART_config.usart_hwflow == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARThandle->USART_config.usart_hwflow == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg |= (( 1 << USART_CR3_CTSE)|( 1 << USART_CR3_RTSE));
	}

	pUSARThandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here
	usart_baudrate(pUSARThandle->pUSARTx, pUSARThandle->USART_config.usart_baud);
}

void usart_deinit(usart_reg *pUSARTx)
{
	 if(pUSARTx == USART1)
	 {
		 USART1_REG_RESET();
	 }
	 else if(pUSARTx == USART2)
	 {
		 USART2_REG_RESET();
	 }
	 else if(pUSARTx == USART6)
	 {
		 USART6_REG_RESET();
	 }
}

void usart_senddata(usart_handle *pUSARThandle, uint8_t *pTxbuffer, uint32_t Len)
{

	uint16_t *pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! usart_flagstatus(pUSARThandle->pUSARTx,USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARThandle->USART_config.usart_wordlen == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxbuffer;
			pUSARThandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARThandle->USART_config.usart_parity == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxbuffer twice
				pTxbuffer++;
				pTxbuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxbuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARThandle->pUSARTx->DR = (*pTxbuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxbuffer++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! usart_flagstatus(pUSARThandle->pUSARTx,USART_FLAG_TC));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void usart_receivedata(usart_handle *pUSARThandle, uint8_t *pRxbuffer, uint32_t Len)
{
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! usart_flagstatus(pUSARThandle->pUSARTx, USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARThandle->USART_config.usart_wordlen == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARThandle->USART_config.usart_parity == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxbuffer) = (pUSARThandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxbuffer two times
				pRxbuffer++;
				pRxbuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxbuffer = (pUSARThandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxbuffer
				 pRxbuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARThandle->USART_config.usart_parity == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxbuffer = pUSARThandle->pUSARTx->DR;
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxbuffer = (uint8_t) 0x7F;

			}

			//increment the pRxbuffer
			pRxbuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataWithIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */


uint8_t usart_senddatait(usart_handle *pUSARThandle,uint8_t *pTxbuffer, uint32_t Len)
{
	uint8_t txstate = pUSARThandle->txstate;

	if(txstate != USART_BUSY_TX)
	{
		pUSARThandle->txlen = Len;
		pUSARThandle->pTxbuffer = pTxbuffer;
		pUSARThandle->txstate = USART_BUSY_TX;

		//Implement the code to enable interrupt for TXE
		pUSARThandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);


		//Implement the code to enable interrupt for TC
		pUSARThandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);


	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs

 */
uint8_t usart_receivedatait(usart_handle *pUSARThandle,uint8_t *pRxbuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARThandle->rxstate;

	if(rxstate != USART_BUSY_RX)
	{
		pUSARThandle->rxlen = Len;
		pUSARThandle->pRxbuffer = pRxbuffer;
		pUSARThandle->rxstate = USART_BUSY_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARThandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rxstate;

}

void usart_irqhandling(usart_handle *pUSARThandle)
{
	uint32_t temp1 , temp2, temp3;
	uint16_t *pdata;
/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARThandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARThandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is   of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARThandle->txstate == USART_BUSY_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! pUSARThandle->txlen )
			{
				//Implement the code to clear the TC flag
				pUSARThandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSARThandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);
				//Reset the application state
				pUSARThandle->txstate = USART_READY;

				//Reset Buffer address to NULL
				pUSARThandle->pTxbuffer = '\0';

				//Reset the length to zero
				pUSARThandle->txlen =0;

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				//USART_ApplicationEventCallback(pUSARThandle,TODO);
				usart_event_callback(pUSARThandle,USART_EVENT_TX_CPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARThandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARThandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARThandle->txstate == USART_BUSY_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARThandle->txlen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARThandle->USART_config.usart_wordlen == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARThandle->pTxbuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARThandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARThandle->USART_config.usart_parity == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxbuffer twice
						pUSARThandle->pTxbuffer++;
						pUSARThandle->pTxbuffer++;

						//Implement the code to decrement the length
						pUSARThandle->txlen--;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARThandle->pTxbuffer++;

						//Implement the code to decrement the length
						pUSARThandle->txlen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARThandle->pUSARTx->DR = (*(pUSARThandle->pTxbuffer)  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARThandle->pTxbuffer++;

					//Implement the code to decrement the length
					pUSARThandle->txlen--;
				}

			}
			if (pUSARThandle->txlen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARThandle->pUSARTx->CR1 &= ~ (1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARThandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARThandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pUSARThandle->rxstate == USART_BUSY_RX)
		{
			//TXE is set so send data
			if(pUSARThandle->rxlen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARThandle->USART_config.usart_wordlen == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARThandle->USART_config.usart_parity == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARThandle->pRxbuffer) = (pUSARThandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARThandle->pRxbuffer++;
						pUSARThandle->pRxbuffer++;

						//Implement the code to decrement the length
						pUSARThandle->rxlen--;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *(pUSARThandle->pRxbuffer) = (pUSARThandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSARThandle->pRxbuffer++;

						 //Implement the code to decrement the length
						 pUSARThandle->rxlen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARThandle->USART_config.usart_parity == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *(pUSARThandle->pRxbuffer) = (uint8_t) (pUSARThandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *(pUSARThandle->pRxbuffer) = (uint8_t) (pUSARThandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARThandle->pRxbuffer++;

					//Implement the code to decrement the length
					pUSARThandle->rxlen--;
				}


			}//if of >0

			if(! pUSARThandle->rxlen)
			{
				//disable the rxne
				pUSARThandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARThandle->rxstate = USART_READY;
				usart_event_callback(pUSARThandle,USART_EVENT_RX_CPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARThandle->pUSARTx->SR &	(1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARThandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARThandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 )
	{
		//Implement the code to clear the CTS flag in SR
		 usart_clearflag(pUSARThandle->pUSARTx, USART_SR_CTS);

		//this interrupt is because of cts
		 usart_event_callback(pUSARThandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARThandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARThandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence

		temp1 = pUSARThandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		usart_event_callback(pUSARThandle,USART_EVENT_CTS);
		//this interrupt is because of idle
		usart_event_callback(pUSARThandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARThandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARThandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .
		usart_clearflag(pUSARThandle->pUSARTx, USART_FLAG_ORE);
		//this interrupt is because of Overrun error
		usart_event_callback(pUSARThandle,USART_ERREVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARThandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARThandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			usart_event_callback(pUSARThandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NE) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			usart_event_callback(pUSARThandle,USART_ERREVENT_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			usart_event_callback(pUSARThandle,USART_ERREVENT_ORE);
		}
	}
	(void)temp3;
}

void usart_irqconfig(uint8_t irq_no, uint8_t state)
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
void usart_irqpriority(uint8_t irq_no, uint32_t priority)
{
	 uint8_t iprx = irq_no/4;
	 uint8_t iprx_section = irq_no%4;
	 uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IPR_BITS);
	 *(NVIC_IPR_BASEADDR + iprx) |= (priority << shift_amount);

}


void usart_peri_ctrl(usart_reg *pUSARTx, uint8_t state)
{
	if(state == ENABLE)
	{
		pUSARTx->CR1 |=	(1<<USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &=	~(1<<USART_CR1_UE);
	}

}
uint8_t usart_flagstatus(usart_reg *pUSARTx , uint32_t flag_name)
{
	if(pUSARTx->SR & flag_name)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void usart_clearflag(usart_reg *pUSARTx, uint16_t flag_name)
{
	pUSARTx->SR &= ~(1 << flag_name);

}
