/*
 * f401_spi.h
 *
 *  Created on: 22-Dec-2021
 *      Author: iamshuvm
 */

#ifndef INC_F401_SPI_H_
#define INC_F401_SPI_H_

#include "stm32f401xx.h"

typedef struct
{
	uint8_t	spi_devmode;
	uint8_t	spi_busconfig;
	uint8_t spi_sclkspeed;
	uint8_t	spi_dff;
	uint8_t	spi_cpol;
	uint8_t	spi_cpha;
	uint8_t ssm;

}spi_config;



typedef struct
{
	spi_config SPI_config;
	spi_reg *pSPIx;
	uint8_t *Txbuffer;
	uint8_t *Rxbuffer;
	uint32_t tx_len;
	uint32_t rx_len;
	uint8_t tx_state;
	uint8_t rx_state;
}spi_handle;


#define SPI_MODE_MASTER 	1
#define SPI_MODE_SLAVE	 	0

#define SPI_BUS_FD			0
#define SPI_BUS_HD			1
#define SPI_BUS_RXONLY		2

#define SPI_SPEED_PSC2		0
#define SPI_SPEED_PSC4		1
#define SPI_SPEED_PSC8		2
#define SPI_SPEED_PSC16		3
#define SPI_SPEED_PSC32		4
#define SPI_SPEED_PSC64		5
#define SPI_SPEED_PSC128	6
#define SPI_SPEED_PSC256	7

#define SPI_DFF_8BIT		0
#define SPI_DFF_16BIT		1

#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1

#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

#define SPI_SSM_DI			0
#define SPI_SSM_EN			1

#define SPI_READY 			0
#define SPI_BUSY_TX 		1
#define SPI_BUSY_RX 		2

#define SPI_EVENT_TX_CPLT	0
#define SPI_EVENT_RX_CPLT	1
#define SPI_EVENT_OVR_ERR	2

#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG		(1 << SPI_SR_FRE)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_UDR_FLAG		(1 << SPI_SR_UDR)



/* SPI init deinit and clock control */
void spi_clockcontrol(spi_reg *pSPIx, uint8_t EN_DIS); 					// spi peripheral clock control
void spi_init(spi_handle *pSPIhandle);									// spi initialization
void spi_deinit(spi_reg *pSPIx);										// spi deinitialization


/* Data send and receiving */
void spi_send(spi_reg *pSPIx, uint8_t *pTxbuffer, uint32_t len);
void spi_receive(spi_reg *pSPIx,uint8_t *pRxbuffer,uint32_t len);

uint8_t spi_send_it(spi_handle *pSPIhandle, uint8_t *pTxbuffer, uint32_t len);
uint8_t spi_receive_it(spi_handle *pSPIhandle, uint8_t *pRxbuffer, uint32_t len);

/* SPI IRQ handling */
void spi_irqconfig(uint8_t irq_no,uint8_t en_dis);						// spi interrupt configuration
void spi_irqpriority(uint8_t irq_no,uint32_t priority);					// spi interrupt priority configuration
void spi_irqhandle(spi_handle *pSPIhandle);										// spi IRQ handling

void spi_peri_ctrl(spi_reg *pSPIx, uint8_t EN_DIS);
void spi_ssiconfig(spi_reg *pSPIx,uint8_t state);
void spi_ssoeconfig(spi_reg *pSPIx,uint8_t state);

uint8_t spi_flagstatus (spi_reg *pSPIx, uint32_t flag_name);
void spi_clear_ovr_flag(spi_reg *pSPIx);
void spi_close_transmission(spi_handle *pSPIhandle);
void spi_close_reception(spi_handle *pSPIhandle);

void spi_event_callback(spi_handle *pSPIhandle,uint8_t app_event);
#endif /* INC_F401_SPI_H_ */
