/*
 * f401_i2c.c
 *
 *  Created on: 27-Dec-2021
 *      Author: iamshuvm
 */

#include "f401_i2c.h"



static void i2c_start(i2c_reg *pI2Cx);			// private helper function
static void address_phase_write(i2c_reg *pI2Cx, uint8_t slave_addr);
static void address_phase_read(i2c_reg *pI2Cx, uint8_t slave_addr);
static void clear_addr_flag(i2c_handle *pI2Chandle );
static void i2c_masterhandle_txeinterrupt(i2c_handle *pI2Chandle);
static void i2c_masterhandle_rxneinterrupt(i2c_handle *pI2Chandle);

static void i2c_start(i2c_reg *pI2Cx)
{
	pI2Cx->CR1 |= (1<< I2C_CR1_START);
}

static void address_phase_write(i2c_reg *pI2Cx, uint8_t slave_addr)
{
	slave_addr = slave_addr << 1;
	slave_addr &= ~ (1);
	pI2Cx->DR = slave_addr;
}

static void address_phase_read(i2c_reg *pI2Cx, uint8_t slave_addr)
{
	slave_addr = slave_addr << 1;
	slave_addr |= 	1;
	pI2Cx->DR = slave_addr;
}



static void clear_addr_flag(i2c_handle *pI2Chandle)
{
	if(pI2Chandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//device in master mode
		if(pI2Chandle->txrx_state == I2C_BUSY_RX)
		{
			if(pI2Chandle->rxsize == 1)
			{
				// disable ack
				i2c_manage_ack(pI2Chandle->pI2Cx, I2C_ACK_DISABLE);
				uint32_t dummy_read = pI2Chandle->pI2Cx->SR1;
				dummy_read = pI2Chandle->pI2Cx->SR2;
				(void)dummy_read;
			}

		}
		else
		{
			uint32_t dummy_read = pI2Chandle->pI2Cx->SR1;
			dummy_read = pI2Chandle->pI2Cx->SR2;
			(void)dummy_read;

		}

	}
	else
	{
		// in slave mode
		uint32_t dummy_read = pI2Chandle->pI2Cx->SR1;
		dummy_read = pI2Chandle->pI2Cx->SR2;
		(void)dummy_read;
	}

	/*

	*/
}

void i2c_clockcontrol(i2c_reg *pI2Cx, uint8_t state) 					// i2c peripheral clock control
{
	if(state == ENABLE)
	 {
		 if(pI2Cx == I2C1)
		 {
			 I2C1_PCLK_EN();
		 }
		 else if(pI2Cx == I2C2)
		 {
			 I2C2_PCLK_EN();
		 }
		 else if(pI2Cx == I2C3)
		 {
			 I2C3_PCLK_EN();
		 }
	 }
	else
	 {
		 if(pI2Cx == I2C1)
		 {
			 I2C1_PCLK_DI();
		 }
		 else if(pI2Cx == I2C2)
		 {
			 I2C2_PCLK_DI();
		 }
		 else if(pI2Cx == I2C3)
		 {
			 I2C3_PCLK_DI();
		 }
	 }
}





void i2c_init(i2c_handle *pI2Chandle)									// i2c initialization
{
	uint16_t tempreg=0;

	// enable the peripheral clock
	i2c_clockcontrol(pI2Chandle->pI2Cx, ENABLE);

	// enable the ACK
	tempreg |= (pI2Chandle->I2C_config.ack_ctrl <<I2C_CR1_ACK);
	pI2Chandle->pI2Cx->CR1 |= tempreg;
	// configure the freq field of CR2
	tempreg = 0;
	tempreg |= rcc_getpclk1_val()/100000U;
	pI2Chandle->pI2Cx->CR2 |= (tempreg & 0x3f);		//	only 5 bits are valid rest are masked out

	// configure device addr
	tempreg = 0;
	tempreg |= (pI2Chandle->I2C_config.device_addr << I2C_OAR1_ADD71);
	tempreg |= (1<<14);		// ??
	pI2Chandle->pI2Cx->OAR1 |= tempreg;

	// CCR -configure the mode (standard or fast)
	uint16_t ccr_val =0;
	tempreg =0;
	if(pI2Chandle->I2C_config.scl_speed <= I2C_SCl_SPEED_SM)
	{
									  // standard mode: T_high = T_low (scl)   => T_scl = 2 * T_high = 2 * (CCR * T_pclk)
		ccr_val=rcc_getpclk1_val()/(2*pI2Chandle->I2C_config.scl_speed);	// =>	CCR = T_scl /(2* T_pclk)
		tempreg |= ccr_val & 0x0fff;									    // =>	CCR = f_pclk /(2* f_scl)
	}
	else
	{
		// FAST MODE
		tempreg |= (1<< I2C_CCR_FS);
		tempreg |= pI2Chandle->I2C_config.fm_duty_cycle << I2C_CCR_DUTY;
		if(pI2Chandle->I2C_config.fm_duty_cycle == I2C_FM_DUTY2)
		{
			ccr_val=rcc_getpclk1_val()/(3*pI2Chandle->I2C_config.scl_speed);	// =>	CCR = f_pclk /(3* f_scl)
			tempreg |= ccr_val & 0x0fff;
		}
		else
		{
			ccr_val=rcc_getpclk1_val()/(25*pI2Chandle->I2C_config.scl_speed);	// =>	CCR = f_pclk /(25* f_scl)
			tempreg |= ccr_val & 0x0fff;
		}
	}
	pI2Chandle->pI2Cx->CCR |= tempreg;

	// TRISE config

	if(pI2Chandle->I2C_config.scl_speed <= I2C_SCl_SPEED_SM)		//	Rp(max)	= Tr/(0.8473*Cb)
	{
		// standard mode:    => 	T_rise(max)  / ( T_pclk) => F_pclk * t_rise(max) + 1
		tempreg =  (rcc_getpclk1_val() /100000U) +1;
   	}
	else
	{
		// FAST MODE
		tempreg =  (rcc_getpclk1_val()*300) /100000000U +1;

	}
	pI2Chandle->pI2Cx->TRISE |= (tempreg & 0x3f);
}

void i2c_deinit(i2c_reg *pI2Cx)
{
	 if(pI2Cx == I2C1)
	 {
		 I2C1_REG_RESET();
	 }
	 else if(pI2Cx == I2C2)
	 {
		 I2C2_REG_RESET();
	 }
	 else if(pI2Cx == I2C3)
	 {
		 I2C3_REG_RESET();
	 }
}

uint8_t i2c_flagstatus (i2c_reg *pI2Cx, uint32_t flag_name)
{
	if(pI2Cx->SR1 & flag_name)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void i2c_manage_ack(i2c_reg *pI2Cx, uint8_t state)
{
	if(state== ENABLE)
	{
		pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
	}


}

void i2c_master_senddata(i2c_handle *pI2Chandle,uint8_t *pTxbuffer, uint32_t len, uint8_t slave_addr, uint8_t rs)
{
	// 1. Generate the START condition
	i2c_start(pI2Chandle->pI2Cx);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits )
	address_phase_write(pI2Chandle->pI2Cx,slave_addr);

	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_ADDR));

	//5. clear the ADDR flag according to its software sequence
	//   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	clear_addr_flag(pI2Chandle);

	//6. send the data until len becomes 0
	while(len>0)
	{
		while(! i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_TXE));
		pI2Chandle->pI2Cx->DR =   *pTxbuffer;
		len--;
		pTxbuffer++;
	}


	//7. when Len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	//   Note: TXE=1 , BTF=1 , means that both SR and DR are empty and next transmission should begin
	//   when BTF=1 SCL will be stretched (pulled to LOW)
	while(! i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_TXE));
	while(! i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_BTF));


	//8. Generate STOP condition and master need not to wait for the completion of stop condition.
	//   Note: generating STOP, automatically clears the BTF
	if(rs == I2C_DISABLE_RS)
		i2c_stop(pI2Chandle->pI2Cx);

}

void i2c_master_receivedata(i2c_handle *pI2Chandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t slave_addr, uint8_t rs)
{

	//1. Generate the START condition
	i2c_start(pI2Chandle->pI2Cx);
	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(! i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_SB));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	address_phase_read(pI2Chandle->pI2Cx, slave_addr);
	//4. wait until address phase is completed by checking the ADDR flag in teh SR1
	while(! i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_ADDR));


	//procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//Disable Acking
		i2c_manage_ack(pI2Chandle->pI2Cx, I2C_ACK_DISABLE);


		//clear the ADDR flag
		clear_addr_flag(pI2Chandle);

		//wait until  RXNE becomes 1
		while(! i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_RXNE));

		//generate STOP condition
		if(rs == I2C_DISABLE_RS)
			i2c_stop(pI2Chandle->pI2Cx);

		//read data in to buffer
		*pRxBuffer = pI2Chandle->pI2Cx->DR;

	}


    //procedure to read data from slave when Len > 1
	if(Len > 1)
	{
		//clear the ADDR flag
		clear_addr_flag(pI2Chandle);

		//read the data until Len becomes zero
		for ( uint32_t i = Len ; i > 0 ; i--)
		{
			//wait until RXNE becomes 1
			while(! i2c_flagstatus(pI2Chandle->pI2Cx, I2C_FLAG_RXNE));

			if(i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				i2c_manage_ack(pI2Chandle->pI2Cx, I2C_ACK_DISABLE);

				//generate STOP condition
				if(rs == I2C_DISABLE_RS)
					i2c_stop(pI2Chandle->pI2Cx);

			}

			//read the data from data register in to buffer
			*pRxBuffer = pI2Chandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

	//re-enable ACKing
	if(pI2Chandle->I2C_config.ack_ctrl == I2C_ACK_ENABLE)
		i2c_manage_ack(pI2Chandle->pI2Cx, I2C_ACK_ENABLE);

}

void i2c_peri_ctrl(i2c_reg *pI2Cx, uint8_t state)
{
	if(state == ENABLE)
	{
		pI2Cx->CR1 |=	(1<<I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &=	~(1<<I2C_CR1_PE);
	}
}

void i2c_irqconfig(uint8_t irq_no,uint8_t state)
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
void i2c_irqpriority(uint8_t irq_no,uint32_t priority)
{
	 uint8_t iprx = irq_no/4;
	 uint8_t iprx_section = irq_no%4;
	 uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IPR_BITS);
	 *(NVIC_IPR_BASEADDR + iprx) |= (priority << shift_amount);
}


uint8_t i2c_senddatait(i2c_handle *pI2Chandle,uint8_t *pTxbuffer, uint32_t len, uint8_t slave_addr, uint8_t rs)
{
	uint8_t busystate = pI2Chandle->txrx_state;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2Chandle->pTxbuffer = pTxbuffer;
		pI2Chandle->tx_len = len;
		pI2Chandle->txrx_state = I2C_BUSY_TX;
		pI2Chandle->dev_addr = slave_addr;
		pI2Chandle->rs = rs;

		//Implement code to Generate START Condition
		i2c_start(pI2Chandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2Chandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Chandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Chandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}

uint8_t i2c_receivedatait(i2c_handle *pI2Chandle,uint8_t *pRxbuffer, uint8_t Len, uint8_t slave_addr, uint8_t rs)
{
	uint8_t busystate = pI2Chandle->txrx_state;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2Chandle->pRxbuffer = pRxbuffer;
		pI2Chandle->rx_len = Len;
		pI2Chandle->txrx_state = I2C_BUSY_RX;
		pI2Chandle->rxsize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2Chandle->dev_addr = slave_addr;
		pI2Chandle->rs = rs;

		//Implement code to Generate START Condition
		i2c_start(pI2Chandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2Chandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Chandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Chandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;

}
static void i2c_masterhandle_txeinterrupt(i2c_handle *pI2Chandle)
{
	if(pI2Chandle->tx_len >0)			//
	{
		pI2Chandle->pI2Cx->DR = *(pI2Chandle->pTxbuffer);
		pI2Chandle->tx_len--;
		pI2Chandle->pTxbuffer++;
	}
}
static void i2c_masterhandle_rxneinterrupt(i2c_handle *pI2Chandle)
{
	if(pI2Chandle->rxsize == 1)			//
	{
		 *pI2Chandle->pRxbuffer = pI2Chandle->pI2Cx->DR ;
		pI2Chandle->rx_len--;
	}
	if(pI2Chandle->rxsize > 1)			//
	{
		if(pI2Chandle->rx_len == 2)
		{
			i2c_manage_ack(pI2Chandle->pI2Cx, I2C_ACK_DISABLE);
		}

		*pI2Chandle->pRxbuffer = pI2Chandle->pI2Cx->DR ;
		pI2Chandle->rx_len--;
		pI2Chandle->pRxbuffer++;
	}
	if(pI2Chandle->rx_len == 0)			//
	{
		if(pI2Chandle->rs == I2C_DISABLE_RS)	//..close the transmission
			i2c_stop(pI2Chandle->pI2Cx);
			// reset all the member elts
		i2c_close_receivedata(pI2Chandle);
		// notify the application
		i2c_event_callback(pI2Chandle,	I2C_EV_RX_CPLT);
	}

}

void i2c_slave_senddata(i2c_reg *pI2Cx, uint8_t data)
{
	pI2Cx->DR =data;

}

uint8_t i2c_slave_receivedata(i2c_reg *pI2Cx)
{
	return (uint8_t )pI2Cx->DR;
}
void i2c_ev_irqhnadling(i2c_handle *pI2Chandle)
{
	//Interrupt handling for both master and slave mode of a device

	uint32_t temp1,temp2,temp3;
	temp1 = (pI2Chandle->pI2Cx->CR2) & (1 << I2C_CR2_ITEVTEN) ;
	temp2 = (pI2Chandle->pI2Cx->CR2) & (1 << I2C_CR2_ITBUFEN) ;
	temp3 = (pI2Chandle->pI2Cx->SR1) & (1 << I2C_SR1_SB) ;

	//1. Handle For interrupt generated by SB event
	//	Note : SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		// SB flag is set
		if(pI2Chandle->txrx_state == I2C_BUSY_TX)
		{
			address_phase_write(pI2Chandle->pI2Cx, pI2Chandle->dev_addr);

		}
		else if(pI2Chandle->txrx_state == I2C_BUSY_RX)
		{
			address_phase_read(pI2Chandle->pI2Cx, pI2Chandle->dev_addr);
		}
	}

	temp3 = (pI2Chandle->pI2Cx->SR1) & (1 << I2C_SR1_ADDR) ;
	//2. Handle For interrupt generated by ADDR event
	//Note : When master mode : Address is sent
	//		 When Slave mode   : Address matched with own address
	if(temp1 && temp3)
	{
		// ADDR flag is set
		clear_addr_flag(pI2Chandle);
	}

	temp3 = (pI2Chandle->pI2Cx->SR1) & (1 << I2C_SR1_BTF) ;
	//3. Handle For interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		// BTF flag is set
		if(pI2Chandle->txrx_state == I2C_BUSY_TX)
		{
			//address_phase_write(pI2Chandle->pI2Cx, pI2Chandle->dev_addr);
			if(pI2Chandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
			{
					// BTF & TXE are set
				if(pI2Chandle->tx_len == 0)
				{
					if(pI2Chandle->rs == I2C_DISABLE_RS)	//..close the transmission
						i2c_stop(pI2Chandle->pI2Cx);
						// reset all the member elts
					i2c_close_senddata(pI2Chandle);
					// notify the application
					i2c_event_callback(pI2Chandle,	I2C_EV_TX_CPLT);
				}
			}
		}
		else if(pI2Chandle->txrx_state == I2C_BUSY_RX)
		{
			;	// do nothing
		}
	}
	temp3 = (pI2Chandle->pI2Cx->SR1) & (1 << I2C_SR1_STOPF) ;
	//4. Handle For interrupt generated by STOPF event
	// Note : Stop detection flag is applicable only slave mode . For master this flag will never be set
	if(temp1 && temp3)
	{
		// STOPF flag is set

		pI2Chandle->pI2Cx->CR1  |= 0x0000;			// write SR1
		//pI2Chandle->pI2Cx->SR1  &= ~(1<< I2C_SR1_STOPF);
		i2c_event_callback(pI2Chandle,	I2C_EV_STOP);
	}
	temp3 = (pI2Chandle->pI2Cx->SR1) & (1 << I2C_SR1_TXE) ;
	//5. Handle For interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		// TXE flag is set
		if(pI2Chandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) // check master or slave mode
		{
			if(pI2Chandle->txrx_state == I2C_BUSY_TX)
			{
				i2c_masterhandle_txeinterrupt(pI2Chandle);
			}
		}
		else
		{
			if(pI2Chandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
				i2c_event_callback(pI2Chandle,	I2C_EV_DATA_REQ);

		}
	}
	temp3 = (pI2Chandle->pI2Cx->SR1) & (1 << I2C_SR1_RXNE) ;
	//6. Handle For interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		// RXNE flag is set
		if(pI2Chandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) // check master or slave mode
		{
			if(pI2Chandle->txrx_state == I2C_BUSY_RX)
			{
				i2c_masterhandle_rxneinterrupt(pI2Chandle);
			}
		}
		else
		{
			if(!(pI2Chandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
				i2c_event_callback(pI2Chandle,	I2C_EV_DATA_RCV);

		}
	}

}
void i2c_err_irqhnadling(i2c_handle *pI2Chandle)
{
	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2Chandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2Chandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2Chandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   i2c_event_callback(pI2Chandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2Chandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2Chandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		i2c_event_callback(pI2Chandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2Chandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2Chandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		i2c_event_callback(pI2Chandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2Chandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2Chandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		i2c_event_callback(pI2Chandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2Chandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

	    //Implement the code to clear the Time out error flag
		pI2Chandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		i2c_event_callback(pI2Chandle,I2C_ERROR_TIMEOUT);
	}

}

void i2c_close_receivedata(i2c_handle *pI2Chandle)
{
	pI2Chandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	pI2Chandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2Chandle->pRxbuffer = NULL;
	pI2Chandle->rx_len = 0;
	pI2Chandle->rxsize = 0;
	pI2Chandle->txrx_state = I2C_READY;

	if(pI2Chandle->I2C_config.ack_ctrl == I2C_ACK_ENABLE)
		i2c_manage_ack(pI2Chandle->pI2Cx, I2C_ACK_ENABLE);

}
void i2c_close_senddata(i2c_handle *pI2Chandle)
{
	pI2Chandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
	pI2Chandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	pI2Chandle->txrx_state = I2C_READY;
	pI2Chandle->pTxbuffer = NULL;
	pI2Chandle->tx_len = 0;

}

void i2c_stop(i2c_reg *pI2Cx)
{
	pI2Cx->CR1 |= (1<< I2C_CR1_STOP);

}
void i2c_slave_enordi_callback(i2c_reg *pI2Cx, uint8_t state)
{
	if(state == ENABLE)
	{
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}
	else
	{
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}
