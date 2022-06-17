/*
 * f401_i2c.h
 *
 *  Created on: 27-Dec-2021
 *      Author: iamshuvm
 */

#ifndef INC_F401_I2C_H_
#define INC_F401_I2C_H_

#include "stm32f401xx.h"

typedef struct
{
	uint32_t scl_speed;
	uint8_t device_addr;
	uint8_t ack_ctrl;
	uint8_t fm_duty_cycle;

}i2c_config;

typedef struct
{
	i2c_config I2C_config;
	i2c_reg *pI2Cx;
	uint8_t *pTxbuffer;
	uint8_t *pRxbuffer;
	uint32_t tx_len;
	uint32_t rx_len;
	uint8_t txrx_state;
	uint8_t dev_addr;
	uint32_t rxsize;
	uint8_t rs;

}i2c_handle;


#define I2C_READY			0
#define I2C_BUSY_RX			1
#define I2C_BUSY_TX			2


#define I2C_SCl_SPEED_SM	100000
#define I2C_SCl_SPEED_FM4K	400000
#define I2C_SCl_SPEED_FM2K	200000


#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

#define I2C_FM_DUTY2		0
#define I2C_FM_DUTY16_9		1


#define I2C_DISABLE_RS		0
#define I2C_ENABLE_RS		1

/*
 * I2C related status flags definitions
 */

#define I2C_FLAG_SB    		( 1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR  		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF   		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_TXE    	( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)

#define I2C_FLAG_STOPF  	( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR   	( 1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO   	( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_OVR   		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT   	( 1 << I2C_SR1_TIMEOUT)


#define I2C_EV_TX_CPLT  	0
#define I2C_EV_RX_CPLT   	1
#define I2C_EV_STOP   		2
#define I2C_ERROR_BERR  	3
#define I2C_ERROR_ARLO  	4
#define I2C_ERROR_AF    	5
#define I2C_ERROR_OVR   	6
#define I2C_ERROR_TIMEOUT 	7
#define I2C_EV_DATA_REQ		8
#define I2C_EV_DATA_RCV		9

/* I2C init deinit and clock control */
void i2c_clockcontrol(i2c_reg *pI2Cx, uint8_t state); 					// i2c peripheral clock control
void i2c_init(i2c_handle *pI2Chandle);									// i2c initialization
void i2c_deinit(i2c_reg *pI2Cx);										// i2c deinitialization

/* Data send and receiving */
void i2c_master_senddata(i2c_handle *pI2Chandle,uint8_t *pTxbuffer, uint32_t len, uint8_t slave_addr, uint8_t r_start);
void i2c_master_receivedata(i2c_handle *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t slave_addr, uint8_t r_start);

uint8_t i2c_senddatait(i2c_handle *pI2Chandle,uint8_t *pTxbuffer, uint32_t len, uint8_t slave_addr, uint8_t r_start);
uint8_t i2c_receivedatait(i2c_handle *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t slave_addr, uint8_t r_start);

void i2c_slave_senddata(i2c_reg *pI2Cx, uint8_t data);
uint8_t i2c_slave_receivedata(i2c_reg *pI2Cx);


void i2c_stop(i2c_reg *pI2Cx);

void i2c_close_receivedata(i2c_handle *pI2Chandle);
void i2c_close_senddata(i2c_handle *pI2Chandle);


/* I2C IRQ handling */
void i2c_irqconfig(uint8_t irq_no,uint8_t state);						// i2c interrupt configuration
void i2c_irqpriority(uint8_t irq_no,uint32_t priority);					// i2c interrupt priority configuration

void i2c_ev_irqhnadling(i2c_handle *pI2CHandle);
void i2c_err_irqhnadling(i2c_handle *pI2CHandle);

void i2c_peri_ctrl(i2c_reg *pI2Cx, uint8_t state);
uint8_t i2c_flagstatus (i2c_reg *pI2Cx, uint32_t flag_name);
void i2c_manage_ack(i2c_reg *pI2Cx, uint8_t state);
void i2c_stop(i2c_reg *pI2Cx);
void i2c_slave_enordi_callback(i2c_reg *pI2Cx, uint8_t state);


void i2c_event_callback(i2c_handle *pI2Chandle,uint8_t app_event);


#endif /* INC_F401_I2C_H_ */
