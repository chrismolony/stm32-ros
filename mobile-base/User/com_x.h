#ifndef _COM_X_H
#define	_COM_X_H
#include "stm32f10x.h"
#define		DINT()		__disable_irq()
#define		EINT()		__enable_irq()
#define USARTzTxBufferSize   64
#define USARTz										USART3
#define USARTz_GPIO								GPIOB
#define USARTz_CLK								RCC_APB1Periph_USART3
#define USARTz_GPIO_CLK						RCC_APB2Periph_GPIOB
#define USARTz_RxPin							GPIO_Pin_11
#define USARTz_TxPin							GPIO_Pin_10
#define USARTz_IRQn								USART3_IRQn
#define USARTz_DR_Base						(uint32_t)(&USART3->DR)
//#define USARTz_DR_Base						USART3_BASE+0x04
#define USARTz_Tx_DMA_Channe			DMA1_Channel2
#define USARTz_Tx_DMA_FLAG				DMA1_FLAG_GL2 //DMA1_FLAG_TC2|DMA1_FLAG_TE2
#define UASRTz_TX_DMA_IRQ					DMA1_Channel2_IRQn

#define USARTz_Rx_DMA_Channe			DMA1_Channel3
#define USARTz_Rx_DMA_FLAG				DMA1_FLAG_GL3 //DMA1_FLAG_TC3 |DMA1_FLAG_TE3
#define UASRTz_RX_DMA_IRQ					DMA1_Channel3_IRQn 

//浮点数与HEX快速获取
typedef	union{
		float fv;
		uint8_t cv[4];
}float_union;

//接收数据结构
typedef	struct{

		float_union		linear_vx;//线速度x
		float_union		linear_vy;//线速度y
		float_union		angular_v;//角速度
		
}rcv_data;

//发送数据结构
typedef	struct{
		
		float_union	x_pos;//x方向坐标
		float_union	y_pos;//y方向坐标
		float_union	x_v;//x方向速度
		float_union	y_v;//y方向速度
		float_union	angular_v;//角速度
		float_union	pose_angular;//角度
	
}send_data;

void com_x_usart_config(void);
void com_x_usart_gpio_config(void);
void com_x_uasrt_nvic_config(void);
void com_x_usart_dma_config(void);
void com_x_usart_dma_tx_over(void);
void com_x_usart_dma_start_tx(uint8_t size);
void com_x_usart_dma_read(void);//DMA 读串口数据 在串口的空闲中断中调用，或者重写该函数，放到其他地方调用
void com_x_usart_rcc_config(void);
//****************外部调用函数************************
void com_x_usart_Init(void);//初始化
int8_t get_data_analyze(uint8_t	*pdata);//接收数据分析
void data_pack(void);//数据打包并发送

#endif
