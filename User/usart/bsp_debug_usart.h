#ifndef __DEBUG_USART_H
#define	__DEBUG_USART_H

#include "stm32f4xx.h"
#include <stdio.h>


//#define USART1_DR_Base  0x40013804		// 0x40013800 + 0x04 = 0x40013804
//#define SENDBUFF_SIZE   5000

#define DEBUG_USART                             USART1
#define DEBUG_USART_CLK                         RCC_APB2Periph_USART1
#define DEBUG_USART_BAUDRATE                    115200

#define DEBUG_USART_RX_GPIO_PORT                GPIOA
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define DEBUG_USART_RX_PIN                      GPIO_Pin_10
#define DEBUG_USART_RX_AF                       GPIO_AF_USART1
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource10

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define DEBUG_USART_TX_PIN                      GPIO_Pin_9
#define DEBUG_USART_TX_AF                       GPIO_AF_USART1
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource9
//////////////////////////////////////  URART2   /////////////////////////////////////
#define DEBUG_USART2                             USART2
#define DEBUG_USART_CLK2                        RCC_APB1Periph_USART2
//#define DEBUG_USART_BAUDRATE2                    100000
#define DEBUG_USART_BAUDRATE2                    115200


#define DEBUG_USART_RX_GPIO_PORT                GPIOA
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define DEBUG_USART_RX_PIN2                      GPIO_Pin_3
#define DEBUG_USART_RX_AF2                       GPIO_AF_USART2
#define DEBUG_USART_RX_SOURCE2                   GPIO_PinSource3

#define DEBUG_USART_TX_GPIO_PORT                GPIOA
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define DEBUG_USART_TX_PIN2                      GPIO_Pin_2
#define DEBUG_USART_TX_AF2                       GPIO_AF_USART2
#define DEBUG_USART_TX_SOURCE2                   GPIO_PinSource2

extern u8 sbus_flag;
extern u8 sbus_buffer[256];
extern 	u8 com_data;
void Uart5_Init(u32 br_num);
void Debug_USART_Config(void);
void Debug_USART_Config2(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void Usart_SendString( USART_TypeDef * pUSARTx, char *str);
void Usart2_Send(unsigned char *DataToSend ,u8 data_num);


//int fputc(int ch, FILE *f);

#endif /* __USART1_H */
