/*
 * f401_usart.h
 *
 *  Created on: 12-Jan-2022
 *      Author: iamshuvm
 */

#ifndef INC_F401_USART_H_
#define INC_F401_USART_H_

#include "stm32f401xx.h"

typedef struct
{
	uint8_t usart_mode;
	uint32_t usart_baud;
	uint8_t usart_stopbit;
	uint8_t usart_wordlen;
	uint8_t usart_parity;
	uint8_t usart_hwflow;

}usart_config;

typedef struct
{
	usart_config USART_config;
	usart_reg *pUSARTx;
	uint8_t *pTxbuffer;
	uint8_t *pRxbuffer;
	uint32_t txlen;
	uint32_t rxlen;
	uint8_t txstate;
	uint8_t rxstate;

}usart_handle;


/*
 *@USART_Mode
 *Possible options for USART_Mode
 */
#define USART_MODE_TX 		0				// refer CR1 -TE and RE
#define USART_MODE_RX 		1
#define USART_MODE_TXRX  	2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_BAUD_1200					1200
#define USART_BAUD_2400					400
#define USART_BAUD_9600					9600
#define USART_BAUD_19200 				19200
#define USART_BAUD_38400 				38400
#define USART_BAUD_57600 				57600
#define USART_BAUD_115200 				115200
#define USART_BAUD_230400 				230400
#define USART_BAUD_460800 				460800
#define USART_BAUD_921600 				921600
#define USART_BAUD_2M 					2000000
#define USART_BAUD_3M 					3000000


/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


#define USART_READY			0
#define USART_BUSY_TX		1
#define USART_BUSY_RX		2

/* USART flags */

#define USART_FLAG_CTS		(1 << USART_SR_CTS)
#define USART_FLAG_TXE		(1 << USART_SR_TXE)
#define USART_FLAG_TC		(1 << USART_SR_TC)
#define USART_FLAG_RXNE		(1 << USART_SR_RXNE)
#define USART_FLAG_ORE		(1 << USART_SR_ORE)
#define USART_FLAG_FE		(1 << USART_SR_FE)
#define USART_FLAG_PE		(1 << USART_SR_PE)

#define 	USART_EVENT_TX_CPLT   	0
#define		USART_EVENT_RX_CPLT   	1
#define		USART_EVENT_IDLE      	2
#define		USART_EVENT_CTS       	3
#define		USART_EVENT_PE        	4
#define		USART_ERREVENT_FE     		5
#define		USART_ERREVENT_NE    	 	6
#define		USART_ERREVENT_ORE    		7

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void usart_clockctrl(usart_reg *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void usart_init(usart_handle *pUSARTHandle);
void usart_deinit(usart_reg *pUSARTx);


/*
 * Data Send and Receive
 */
void usart_senddata(usart_handle *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
void usart_receivedata(usart_handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t usart_senddatait(usart_handle *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t usart_receivedatait(usart_handle *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void usart_irqconfig(uint8_t IRQNumber, uint8_t EnorDi);
void usart_irqpriority(uint8_t IRQNumber, uint32_t IRQPriority);
void usart_irqhandling(usart_handle *pHandle);

/*
 * Other Peripheral Control APIs
 */
void usart_baudrate(usart_reg *pUSARTx, uint32_t baud_rate);
void usart_peri_ctrl(usart_reg *pUSARTx, uint8_t state);
uint8_t usart_flagstatus(usart_reg *pUSARTx , uint32_t flag_name);
void usart_clearflag(usart_reg *pUSARTx, uint16_t flag_status);

/*
 * Application callback
 */
void usart_event_callback(usart_handle *pUSARTHandle,uint8_t AppEv);

#endif /* INC_F401_USART_H_ */
