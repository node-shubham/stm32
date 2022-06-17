/*
 * f401_spi.c
 *
 *  Created on: 22-Dec-2021
 *      Author: iamshuvm
 */
#include "f401_spi.h"

static void spi_tx_interrupt_handle(spi_handle *pSPIhandle);  // private helper func
static void spi_rx_interrupt_handle(spi_handle *pSPIhandle);
static void spi_ovr_interrupt_handle(spi_handle *pSPIhandle);

void spi_clockcontrol(spi_reg *pSPIx, uint8_t state)		// spi peripheral clock control
{
	if(state == ENABLE)
	 {
		 if(pSPIx == SPI1)
		 {
			 SPI1_PCLK_EN();
		 }
		 else if(pSPIx == SPI2)
		 {
			 SPI2_PCLK_EN();
		 }
		 else if(pSPIx == SPI3)
		 {
			 SPI3_PCLK_EN();
		 }
		 else if(pSPIx == SPI4)
		 {
			 SPI4_PCLK_EN();
		 }

	 }
	else
	 {
		 if(pSPIx == SPI1)
		 {
			 SPI1_PCLK_DI();
		 }
		 else if(pSPIx == SPI2)
		 {
			 SPI2_PCLK_DI();
		 }
		 else if(pSPIx == SPI3)
		 {
			 SPI3_PCLK_DI();
		 }
		 else if(pSPIx == SPI4)
		 {
			 SPI4_PCLK_DI();
		 }
	 }
}

void spi_init(spi_handle *pSPIhandle)
{
	uint16_t tempreg =0;
	// enable peripheral clock
	spi_clockcontrol(pSPIhandle->pSPIx, ENABLE);

	// configuring device mode
	tempreg	|= ((pSPIhandle->SPI_config.spi_devmode) << 2);
	//tempreg |= pSPIHandle->SPI_config.spi_devmode << SPI_CR1_MSTR ;


	// configure BIDIR mode
	if (pSPIhandle->SPI_config.spi_busconfig == SPI_BUS_FD)
	{
		tempreg	&= ~(1 << 15);// bidi disabled
	}
	else if(pSPIhandle->SPI_config.spi_busconfig == SPI_BUS_HD)
	{
		tempreg	|= (1 << 15);  // bidi enabled
	}
	else if(pSPIhandle->SPI_config.spi_busconfig == SPI_BUS_RXONLY)
	{
		tempreg	&= ~(1 << 15);
		tempreg	|= (1 << 10); //bidi disabled & rxonly bit high
	}

	// configuring the speed
	tempreg |= (pSPIhandle->SPI_config.spi_sclkspeed << 3);
	// configuring the DFF
	tempreg |= (pSPIhandle->SPI_config.spi_dff << 11);
	// configuring the CPOL
	tempreg |= (pSPIhandle->SPI_config.spi_cpol << 1);
	// configuring the CPHA
	tempreg |= (pSPIhandle->SPI_config.spi_cpha << 0);
	// configuring the slave selection management
	tempreg |= (pSPIhandle->SPI_config.ssm << 9);


	pSPIhandle->pSPIx->CR1 |= tempreg;
}

void spi_deinit(spi_reg *pSPIx)
{
	 if(pSPIx == SPI1)
	 {
		 SPI1_REG_RESET();
	 }
	 else if(pSPIx == SPI2)
	 {
		 SPI2_REG_RESET();
	 }
	 else if(pSPIx == SPI3)
	 {
		 SPI3_REG_RESET();
	 }
	 else if(pSPIx == SPI4)
	 {
		 SPI4_REG_RESET();
	 }
}

//void spi_peri_ctrl(spi_handle *pSPIhandle, uint8_t state)
void spi_peri_ctrl(spi_reg *pSPIx, uint8_t state)
{
	if(state == ENABLE)
	{
		//pSPIhandle->pSPIx->CR1 |=	(1<<SPI_CR1_SPE);
		pSPIx->CR1 |=	(1<<SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &=	~(1<<SPI_CR1_SPE);
	}
}
void spi_ssiconfig(spi_reg *pSPIx,uint8_t state)
{
	if(state == ENABLE)
	{
		//pSPIhandle->pSPIx->CR1 |=	(1<<SPI_CR1_SPE);
		pSPIx->CR1 |=	(1<<SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &=	~(1<<SPI_CR1_SSI);
	}
}

void spi_ssoeconfig(spi_reg *pSPIx,uint8_t state)
{
	if(state == ENABLE)
	{
		pSPIx->CR2 |=	(1<< SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &=	~(1<<SPI_CR2_SSOE);
	}
}

void spi_send(spi_reg *pSPIx, uint8_t *pTxbuffer, uint32_t len)
{
	while(len>0)				// blocking method (polling based)
	{
		while(spi_flagstatus(pSPIx,SPI_TXE_FLAG)== FLAG_RESET);									//while((pSPIx->SR>>1)&0x01 != 1);

		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			pSPIx->DR = *((uint16_t *)pTxbuffer);			// 16 bit mode
			len-=2;
			(uint16_t *)pTxbuffer++;
		}
		else
		{
			pSPIx->DR = *pTxbuffer;							// 8 bit mode
			len-=1;
			pTxbuffer++;
		}
	}

}
void spi_receive(spi_reg *pSPIx,uint8_t *pRxbuffer,uint32_t len)
{
	while(len>0)				// blocking method (polling based)
	{
		while(spi_flagstatus(pSPIx,SPI_RXNE_FLAG)== FLAG_RESET);	//while((pSPIx->SR>>1)&0x01 != 1);

		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			*((uint16_t *)pRxbuffer)= pSPIx->DR;			// 16 bit mode Rx
			len-=2;
			(uint16_t *)pRxbuffer++;
		}
		else
		{
			*pRxbuffer= pSPIx->DR ;
			len-=1;
			pRxbuffer++;
		}
	}
}

uint8_t spi_flagstatus (spi_reg *pSPIx, uint32_t flag_name)
{
	if(pSPIx->SR & flag_name)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void spi_irqconfig(uint8_t irq_no,uint8_t state)
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

void spi_irqpriority(uint8_t irq_no,uint32_t priority)
{
	 uint8_t iprx = irq_no/4;
	 uint8_t iprx_section = irq_no%4;
	 uint8_t shift_amount = (8 * iprx_section) + (8 - NO_IPR_BITS);
	 *(NVIC_IPR_BASEADDR + iprx) |= (priority << shift_amount);
}



uint8_t spi_send_it(spi_handle *pSPIhandle, uint8_t *pTxbuffer, uint32_t len)
{
	uint8_t state = pSPIhandle->tx_state;
	if(state != SPI_BUSY_TX)
	{
		pSPIhandle->Txbuffer = pTxbuffer;
		pSPIhandle->tx_len = len;
		pSPIhandle->tx_state = SPI_BUSY_TX;
		pSPIhandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return state;
}
uint8_t spi_receive_it(spi_handle *pSPIhandle, uint8_t *pRxbuffer, uint32_t len)
{
	uint8_t state = pSPIhandle->tx_state;
	if(state != SPI_BUSY_RX)
	{
		pSPIhandle->Rxbuffer = pRxbuffer;
		pSPIhandle->rx_len = len;
		pSPIhandle->rx_state = SPI_BUSY_RX;
		pSPIhandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	return state;

}

void spi_irqhandle(spi_handle *pSPIhandle)
{
	uint8_t temp1, temp2;
	temp1 = (pSPIhandle->pSPIx->SR & (1<<SPI_SR_TXE));
	temp2 = (pSPIhandle->pSPIx->CR2 & (1<<SPI_CR2_TXEIE));
	if(temp1 & temp2)	// handle txe interrupt
	{
		spi_tx_interrupt_handle(pSPIhandle);
	}

	temp1 = (pSPIhandle->pSPIx->SR & (1<<SPI_SR_RXNE));
	temp2 = (pSPIhandle->pSPIx->CR2 & (1<<SPI_CR2_RXNEIE));
	if(temp1 & temp2)	// handle rxne interrupt
	{
		spi_rx_interrupt_handle(pSPIhandle);
	}

	temp1 = (pSPIhandle->pSPIx->SR & (1<<SPI_SR_OVR));
	temp2 = (pSPIhandle->pSPIx->CR2 & (1<<SPI_CR2_ERRIE));
	if(temp1 & temp2)	// handle error interrupt
	{
		spi_ovr_interrupt_handle(pSPIhandle);
	}
}

static void spi_tx_interrupt_handle(spi_handle *pSPIhandle)  // private helper function
{
	if(pSPIhandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		pSPIhandle->pSPIx->DR = *((uint16_t *)pSPIhandle->Txbuffer);			// 16 bit mode
		pSPIhandle->tx_len--;
		pSPIhandle->tx_len--;
		(uint16_t *)pSPIhandle->Txbuffer++;
	}
	else
	{
		pSPIhandle->pSPIx->DR = *(pSPIhandle->Txbuffer);							// 8 bit mode
		pSPIhandle->tx_len--;
		(pSPIhandle->Txbuffer)++;
	}
	if(! pSPIhandle->tx_len )
	{
		spi_close_transmission(pSPIhandle);
	}

}
static void spi_rx_interrupt_handle(spi_handle *pSPIhandle)
{
	if(pSPIhandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		*((uint16_t *)pSPIhandle->Rxbuffer)= pSPIhandle->pSPIx->DR;			// 16 bit mode Rx
		pSPIhandle->rx_len--;
		pSPIhandle->rx_len--;
		(uint16_t *)pSPIhandle->Rxbuffer--;
	}
	else
	{
		*pSPIhandle->Rxbuffer = pSPIhandle->pSPIx->DR ;
		pSPIhandle->rx_len--;
		pSPIhandle->Rxbuffer--;
	}
	if(! pSPIhandle->rx_len )
	{
		spi_close_reception(pSPIhandle);
	}
}
static void spi_ovr_interrupt_handle(spi_handle *pSPIhandle)
{
	uint8_t temp;	 //clear ovr flag
	if(pSPIhandle->tx_state != SPI_BUSY_TX)
	{
		temp = pSPIhandle->pSPIx->DR;  // ?
		temp = pSPIhandle->pSPIx->SR;	// ?
	}
	(void)temp;
		// inform the application
	spi_event_callback(pSPIhandle,SPI_EVENT_OVR_ERR);
}

void spi_clear_ovr_flag(spi_reg *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
void spi_close_transmission(spi_handle *pSPIhandle)
{
	pSPIhandle->pSPIx->CR2 &= ~(1<<SPI_CR2_TXEIE);
	pSPIhandle->tx_len =0;
	pSPIhandle->Txbuffer = NULL;
	pSPIhandle->tx_state = SPI_READY;
	spi_event_callback(pSPIhandle,SPI_EVENT_TX_CPLT);
}
void spi_close_reception(spi_handle *pSPIhandle)
{
	pSPIhandle->pSPIx->CR2 &= ~(1<<SPI_CR2_RXNEIE);
	pSPIhandle->rx_len =0;
	pSPIhandle->Rxbuffer = NULL;
	pSPIhandle->rx_state = SPI_READY;
	spi_event_callback(pSPIhandle,SPI_EVENT_TX_CPLT);
}

__weak void spi_event_callback(spi_handle *pSPIhandle,uint8_t app_event)
{

}
