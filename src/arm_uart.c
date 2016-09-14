/**
	This file is part of martink project.

	martink firmware project is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	martink firmware is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with martink firmware.  If not, see <http://www.gnu.org/licenses/>.

	Author: Martin K. Schröder
	Email: info@fortmax.se
	Github: https://github.com/mkschreder
*/

#include <inttypes.h>
#include <string.h>

#include <utype/cbuf.h>

#include "arm_uart.h"

#define UART_RX_BUFFER_SIZE 64
#define UART_TX_BUFFER_SIZE 128
#define DEV_COUNT sizeof(_devices)/sizeof(struct uart_device)

struct arm_uart {
	USART_TypeDef *dev; 
	GPIO_TypeDef *gpio; 
	int rcc_gpio; 
	int rcc_id; 
	int apb_id; 
	int out_pins, in_pins; 
	int irq; 
	
	struct cbuf rx_buf, tx_buf; 
	char rx_buffer[UART_RX_BUFFER_SIZE]; 
	char tx_buffer[UART_TX_BUFFER_SIZE]; 
}; 

static const struct arm_uart _devices[] = {
	{
		.dev = USART1, 
		.gpio = GPIOA, 
		.rcc_gpio = RCC_APB2Periph_GPIOA, 
		.rcc_id = RCC_APB2Periph_USART1, 
		.apb_id = 2, 
		.out_pins = GPIO_Pin_9, 
		.in_pins = GPIO_Pin_10, 
		.irq = USART1_IRQn
	}, 
	{
		.dev = USART2, 
		.gpio = GPIOA, 
		.rcc_gpio = RCC_APB2Periph_GPIOA, 
		.rcc_id = RCC_APB1Periph_USART2, 
		.apb_id = 1, 
		.out_pins = GPIO_Pin_2, 
		.in_pins = GPIO_Pin_3, 
		.irq = USART2_IRQn
	}, 
	{
		.dev = USART3, 
		.gpio = GPIOB, 
		.rcc_gpio = RCC_APB2Periph_GPIOB, 
		.rcc_id = RCC_APB1Periph_USART3, 
		.apb_id = 1, 
		.out_pins = GPIO_Pin_10, 
		.in_pins = GPIO_Pin_11, 
		.irq = USART3_IRQn
	}
}; 

int8_t arm_uart_init(struct arm_uart *self, uint32_t baud, uint8_t *tx_buffer, uint8_t tx_size, uint8_t *rx_buffer, uint8_t rx_size){
	USART_InitTypeDef usartConfig;
	
	USART_TypeDef *dev = self->dev; 
	
	cbuf_init(&self->rx_buf, (char*)rx_buffer, rx_size); 
	cbuf_init(&self->tx_buf, (char*)tx_buffer, tx_size); 
	
	USART_DeInit(dev); 
	
	if(conf->apb_id == 2){
		RCC_APB2PeriphClockCmd(conf->rcc_id, ENABLE);
	} else if(conf->apb_id == 1){
		RCC_APB1PeriphClockCmd(conf->rcc_id, ENABLE);
	} 

	RCC_APB2PeriphClockCmd(conf->rcc_gpio | RCC_APB2Periph_AFIO, ENABLE);
	
	USART_Cmd(dev, ENABLE);

	usartConfig.USART_BaudRate = baud;
	usartConfig.USART_WordLength = USART_WordLength_8b;
	usartConfig.USART_StopBits = USART_StopBits_1;
	usartConfig.USART_Parity = USART_Parity_No;
	usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartConfig.USART_HardwareFlowControl =
			 USART_HardwareFlowControl_None;
	USART_Init(dev, &usartConfig);
	
	GPIO_InitTypeDef gpioConfig;

	// configure tx pin
	gpioConfig.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioConfig.GPIO_Pin = conf->out_pins;
	gpioConfig.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(conf->gpio, &gpioConfig);

	// configure rx pin
	gpioConfig.GPIO_Mode = GPIO_Mode_IPU;
	gpioConfig.GPIO_Pin = conf->in_pins;
	GPIO_Init(conf->gpio, &gpioConfig);
	
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the USARTx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = conf->irq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  
	USART_ITConfig(dev, USART_IT_RXNE, ENABLE);
	USART_ITConfig(dev, USART_IT_TXE, ENABLE);
	
	return 0; 
}

static void arm_uart_deinit(struct arm_uart *self){
	USART_DeInit(self->dev); 
}

static int8_t uart_set_baudrate(struct arm_uart *self, uint32_t baud){
	USART_InitTypeDef usartConfig;
	
	USART_Cmd(self->dev, DISABLE);

	usartConfig.USART_BaudRate = baud;
	usartConfig.USART_WordLength = USART_WordLength_8b;
	usartConfig.USART_StopBits = USART_StopBits_1;
	usartConfig.USART_Parity = USART_Parity_No;
	usartConfig.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartConfig.USART_HardwareFlowControl =
			 USART_HardwareFlowControl_None;
	USART_Init(self->dev, &usartConfig);
	
	USART_Cmd(self->dev, ENABLE);
	
	return 0; 
}

#if 0
int uart_getc(uint8_t dev_id){
	uint8_t count = sizeof(_devices) / sizeof(_devices[0]); 
	if(dev_id >= count) return -EAGAIN; 
	USART_TypeDef *dev = _devices[dev_id].dev; 

	uint16_t ret = SERIAL_NO_DATA; 
	USART_ITConfig(dev, USART_IT_RXNE, DISABLE);
	
	// no need for critical section because cbuffer is threadsafe
	if(cbuf_get_waiting(&rx_buffers[dev_id]) > 0) { ret = cbuf_get(&rx_buffers[dev_id]); }
	
	USART_ITConfig(dev, USART_IT_RXNE, ENABLE);
	return ret; 
}
#endif

int8_t arm_uart_putc(struct arm_uart *self, uint8_t ch){
	USART_TypeDef *dev = self->dev; 

	// TODO: use a mutex	
	while(cbuf_is_full(&self->tx_buf)); 
	
	USART_ITConfig(dev, USART_IT_TXE, DISABLE);
	cbuf_put(&self->tx_buf, ch);
	USART_ITConfig(dev, USART_IT_TXE, ENABLE);
	
	return 0; 
}

static void USART_Handler(USART_TypeDef *dev, struct cbuf *rx_buf, struct cbuf *tx_buf){
	if(USART_GetITStatus(dev, USART_IT_RXNE) != RESET){
		USART_ClearITPendingBit(dev, USART_IT_RXNE);
		unsigned char ch = USART_ReceiveData(dev) & 0xff;
		cbuf_put_isr(rx_buf, ch); 
	}
	if(USART_GetITStatus(dev, USART_IT_TXE) != RESET){
		USART_ClearITPendingBit(dev, USART_IT_TXE);
		if(!cbuf_is_empty(tx_buf))
			USART_SendData(dev, cbuf_get(tx_buf)); 
		else
			USART_ITConfig(dev, USART_IT_TXE, DISABLE);
	}
}

void USART1_IRQHandler(void);
void USART1_IRQHandler(void){
	USART_Handler(USART1, &_devices[0].rx_buf, &_devices[0].tx_buf); 
}

void USART2_IRQHandler(void);
void USART2_IRQHandler(void)
{
	USART_Handler(USART2, &_devices[1].rx_buf, &_devices[1].tx_buf); 
}

void USART3_IRQHandler(void);
void USART3_IRQHandler(void)
{	
	USART_Handler(USART3, &_devices[2].rx_buf, &_devices[2].tx_buf); 
}

static uint16_t uart_waiting(struct arm_uart *self){
	return cbuf_get_waiting(&self->rx_buf); 
}
