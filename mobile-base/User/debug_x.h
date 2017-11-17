#ifndef _debug_X_H
#define	_debug_X_H
#include "stm32f10x.h"
/*************************************************************
@StevenShi 
**************************************************************/

#define USARTm										USART1
#define USARTm_GPIO								GPIOA
#define USARTm_CLK								RCC_APB2Periph_USART1
#define USARTm_GPIO_CLK						RCC_APB2Periph_GPIOA
#define USARTm_RxPin							GPIO_Pin_10
#define USARTm_TxPin							GPIO_Pin_9
#define USARTm_IRQn								USART1_IRQn
#define USARTm_DR_Base						(uint32_t)(&USART1->DR)
//#define USARTm_DR_Base						USART1_BASE+0x04
#define USARTm_Tx_DMA_Channe			DMA1_Channel4
#define USARTm_Tx_DMA_FLAG				DMA1_FLAG_GL4 //DMA1_FLAG_TC4|DMA1_FLAG_TE4
#define UASRTm_TX_DMA_IRQ					DMA1_Channel4_IRQn

#define USARTm_Rx_DMA_Channe			DMA1_Channel5
#define USARTm_Rx_DMA_FLAG				DMA1_FLAG_GL5 //DMA1_FLAG_TC5 |DMA1_FLAG_TE5
#define UASRTm_RX_DMA_IRQ					DMA1_Channel5_IRQn 



#define USARTm_Tx_BUFFER_SIZE	256 //发送缓存长度
#define USARTm_Rx_BUFFER_SIZE	32  //接收缓存长度

//队列-用于printf函数要发送的内容
typedef struct { 	
	uint8_t	USARTm_Tx_Buffer[USARTm_Tx_BUFFER_SIZE]; //队列空间
	__IO	uint32_t	USARTm_Tx_PTR_HEAD; //队列头指针
	__IO	uint32_t	USARTm_Tx_PTR_TAIL; //队列尾指针
	__IO	uint32_t	USARTm_Tx_COUNTER; 	//当前队列长度
	__IO	uint8_t		USARTm_Tx_BUFFER_FULL;  //队列满标志
}USARTm_Tx_Buffer_TypeDef;  



//*****************内部函数************************
void debug_x_usart_config(void);
void debug_x_usart_gpio_config(void);
void debug_x_uasrt_nvic_config(void);
void debug_x_usart_dma_rx_config(void);//DMA接收配置
void debug_x_usart_dma_tx_config(uint32_t memoryBaseAddr, uint8_t sendBufferSize);//DMA发送配置 
void debug_x_usart_dma_read(void);//DMA 读串口数据 在串口的空闲中断中调用，或者重写该函数，放到其他地方调用
void debug_x_usart_rcc_config(void);

//****************外部调用函数************************
void debug_x_usart_Init(void);//初始化
void debug_x_usart_dma_start_tx(uint32_t memoryBaseAddr,uint32_t SendBufferSize);//使用DMA发送时给出发送缓存首地址和长度即可
int debug_x_usart_dma_ctl(void);//该函数用于printf的实现，在串口输出打印调试信息
uint32_t debug_x_usart_In_Queue(uint8_t ch);//将数据放入队列
uint32_t debug_x_usart_Out_Queue(uint8_t *ch);//将数据读出队列
#endif
//@StevenShi 
/**************************************************************************************************/
