
#include "com_x.h"
#include "debug_x.h"

#include "stdio.h"
#include "string.h"



uint8_t USARTzTxBuffer[USARTzTxBufferSize];
uint8_t USARTzRxBuffer[USARTzTxBufferSize];
uint8_t USARTzRxBufferD[USARTzTxBufferSize];

send_data	com_x_send_data;//数据发送
rcv_data	com_x_rcv_data;//数据接收
 

//115200 8bit 1stop no parity
void com_x_usart_config(void)
{
//  USARTz configuration
/* Configure USARTz */
	USART_InitTypeDef USARTz_InitStructure;
  USARTz_InitStructure.USART_BaudRate = 115200;
  USARTz_InitStructure.USART_WordLength = USART_WordLength_8b;//
  USARTz_InitStructure.USART_StopBits = USART_StopBits_1;
  USARTz_InitStructure.USART_Parity = USART_Parity_No;
  USARTz_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USARTz_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USARTz, &USARTz_InitStructure);
	
	/* Enable USART3 Receive  interrupts */
  USART_ITConfig(USARTz, USART_IT_IDLE, ENABLE); //开启串口空闲IDLE中断
  /* Enable USARTz DMA TX request */
  USART_DMACmd(USARTz, USART_DMAReq_Tx, ENABLE);
	/* Enable USARTz DMA RX request */
	USART_DMACmd(USARTz, USART_DMAReq_Rx, ENABLE);
  /* Enable USARTz */
  USART_Cmd(USARTz, ENABLE);
  /* Enable USARTz DMA TX Channel */
	USART_DMACmd(USARTz, USART_DMAReq_Tx,ENABLE);// DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);
 
	/* Enable USARTz DMA RX Channel */
	USART_DMACmd(USARTz, USART_DMAReq_Rx,ENABLE);//DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE);
	
}
void com_x_usart_gpio_config(void)
{

  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure USARTz Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//
  GPIO_InitStructure.GPIO_Pin = USARTz_RxPin;
  GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);  
  
  /* Configure USARTz Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用形式的推挽输出
  GPIO_InitStructure.GPIO_Pin = USARTz_TxPin;
  GPIO_Init(USARTz_GPIO, &GPIO_InitStructure);
	
}

void com_x_uasrt_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = (uint8_t)UASRTz_TX_DMA_IRQ;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable the USARTz Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USARTz_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void com_x_usart_dma_config(void)
{
  DMA_InitTypeDef DMA_InitStructure;

/* USARTz_Tx_DMA_Channel  */
  DMA_Cmd(USARTz_Tx_DMA_Channe,DISABLE);//stop dma
	DMA_DeInit(USARTz_Tx_DMA_Channe);//恢复缺省配置
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTz_DR_Base;//串口发送数据寄存器
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTzTxBuffer;//发送缓冲首地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//设置外设为目标
  DMA_InitStructure.DMA_BufferSize = USARTzTxBufferSize;//需要发送的字节数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整       
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整              
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节        
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级                
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式 
	DMA_Init(USARTz_Tx_DMA_Channe, &DMA_InitStructure);//写入配置	
	DMA_ClearFlag(USARTz_Tx_DMA_FLAG);    //清除DMA所有标志                          
	//DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE); 
	DMA_ITConfig(USARTz_Tx_DMA_Channe, DMA_IT_TC, ENABLE);  //开启DMA发送通道中断  
	
/*USARTz_Rx_DMA_Channe*/
	DMA_Cmd(USARTz_Rx_DMA_Channe,DISABLE);//stop dma
	DMA_DeInit(USARTz_Rx_DMA_Channe);//恢复缺省配置
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTz_DR_Base;//设置串口接收数据寄存器
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTzRxBuffer;//接收缓存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//设置外设为数据源
  DMA_InitStructure.DMA_BufferSize = USARTzTxBufferSize;//需要接收的字节数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整       
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整              
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节        
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级                
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式 
	DMA_Init(USARTz_Rx_DMA_Channe, &DMA_InitStructure);//写入配置	
	DMA_ClearFlag(USARTz_Rx_DMA_FLAG);    //清除DMA所有标志                          
	DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE); //开启DMA接收通道
	   
}
//************************************DMA发送********************************
void DMA1_Channel2_IRQHandler(void)//中断向量定义在startup_stm32f10x.s文件中
{
    if(DMA_GetITStatus(DMA1_FLAG_TC2))
    {
        com_x_usart_dma_tx_over();
    }
}
void com_x_usart_dma_tx_over(void)
{
    DMA_ClearFlag(USARTz_Tx_DMA_FLAG);    //清除DMA所有标志    
    DMA_Cmd(USARTz_Tx_DMA_Channe, DISABLE);  //关闭DMA发送通道
    
}
 
void com_x_usart_dma_start_tx(uint8_t size)
{
    USARTz_Tx_DMA_Channe->CNDTR = (uint16_t)size; //重新赋值 指定发送缓存长度
    DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);  //开启DMA发送      
}
//**************************************************************************

void USART3_IRQHandler(void)//中断向量定义在startup_stm32f10x.s中
{
    
	if(USART_GetITStatus(USARTz, USART_IT_IDLE) != RESET)  
    {
        com_x_usart_dma_read();
				USART_ReceiveData( USARTz );
    }
}
void com_x_usart_dma_read(void)
{
    uint8_t rxcounter;
		uint8_t i;
		DMA_Cmd(USARTz_Rx_DMA_Channe, DISABLE);    //关闭DMA防止干扰   
    DMA_ClearFlag( USARTz_Rx_DMA_FLAG );   //清除标志位       
    rxcounter= USARTzTxBufferSize - DMA_GetCurrDataCounter(USARTz_Rx_DMA_Channe);//获取接收到的字节数 
    USARTz_Rx_DMA_Channe->CNDTR = USARTzTxBufferSize; //重新赋值计数值   
    memset(USARTzRxBufferD,0,sizeof(USARTzRxBufferD));
		printf("Received data:");
	  for(i=0;i<rxcounter;i++){
			USARTzRxBufferD[i] = USARTzRxBuffer[i];//获取接收到的数据，存入数组RxBufferD中
			
			printf("0x%02x",USARTzRxBufferD[i]);
		}
		printf("\n\r");
		//分析收到的数据包
		//DINT();
		if(get_data_analyze(USARTzRxBufferD) != 0)
			printf("data analyze error\n\r");
		//EINT();
		for(i=0;i<rxcounter;i++)
			USARTzRxBuffer[i] = 0;//clear Rx buffer
		DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE);  //DMA开启 等待下一帧数据     
    
   
   
}
void com_x_usart_rcc_config(void)
{
/* DMA clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
  RCC_APB2PeriphClockCmd( USARTz_GPIO_CLK  |RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(USARTz_CLK,ENABLE);

}
void com_x_usart_Init(void)
{
	
	com_x_usart_rcc_config();
	com_x_uasrt_nvic_config();
	com_x_usart_gpio_config();
	com_x_usart_dma_config();
	com_x_usart_config();
}
//测试
void data_pack(void)
{
	
	com_x_send_data.x_pos.fv = 2.68;//x坐标
	com_x_send_data.y_pos.fv = 3.96;//y坐标
	com_x_send_data.x_v.fv	= 0.6;//x方向速度
	com_x_send_data.y_v.fv = 0.0;//y 方向速度
	com_x_send_data.angular_v.fv = 2.0;//角速度 绕z轴
	com_x_send_data.pose_angular.fv = 1.0;//yaw偏航角 
	
	USARTzTxBuffer[0] = 0xaa;
	USARTzTxBuffer[1] = 0xaa;
	
	USARTzTxBuffer[2] = com_x_send_data.x_pos.cv[0];
	USARTzTxBuffer[3] = com_x_send_data.x_pos.cv[1];
	USARTzTxBuffer[4] = com_x_send_data.x_pos.cv[2];
	USARTzTxBuffer[5] = com_x_send_data.x_pos.cv[3];
	
	USARTzTxBuffer[6] = com_x_send_data.y_pos.cv[0];
	USARTzTxBuffer[7] = com_x_send_data.y_pos.cv[1];
	USARTzTxBuffer[8] = com_x_send_data.y_pos.cv[2];
	USARTzTxBuffer[9] = com_x_send_data.y_pos.cv[3];
	
	USARTzTxBuffer[10] = com_x_send_data.x_v.cv[0];
	USARTzTxBuffer[11] = com_x_send_data.x_v.cv[1];
	USARTzTxBuffer[12] = com_x_send_data.x_v.cv[2];
	USARTzTxBuffer[13] = com_x_send_data.x_v.cv[3];
	
	USARTzTxBuffer[14] = com_x_send_data.y_v.cv[0];
	USARTzTxBuffer[15] = com_x_send_data.y_v.cv[1];
	USARTzTxBuffer[16] = com_x_send_data.y_v.cv[2];
	USARTzTxBuffer[17] = com_x_send_data.y_v.cv[3];
	
	USARTzTxBuffer[18] = com_x_send_data.angular_v.cv[0];
	USARTzTxBuffer[19] = com_x_send_data.angular_v.cv[1];
	USARTzTxBuffer[20] = com_x_send_data.angular_v.cv[2];
	USARTzTxBuffer[21] = com_x_send_data.angular_v.cv[3];
	
	USARTzTxBuffer[22] = com_x_send_data.pose_angular.cv[0];
	USARTzTxBuffer[23] = com_x_send_data.pose_angular.cv[1];
	USARTzTxBuffer[24] = com_x_send_data.pose_angular.cv[2];
	USARTzTxBuffer[25] = com_x_send_data.pose_angular.cv[3];
	
	USARTzTxBuffer[26] = USARTzTxBuffer[2]^USARTzTxBuffer[3]^USARTzTxBuffer[4]^USARTzTxBuffer[5]^USARTzTxBuffer[6]^
												USARTzTxBuffer[7]^USARTzTxBuffer[8]^USARTzTxBuffer[9]^USARTzTxBuffer[10]^USARTzTxBuffer[11]^
												USARTzTxBuffer[12]^USARTzTxBuffer[13]^USARTzTxBuffer[14]^USARTzTxBuffer[15]^USARTzTxBuffer[16]^
												USARTzTxBuffer[17]^USARTzTxBuffer[18]^USARTzTxBuffer[19]^USARTzTxBuffer[20]^USARTzTxBuffer[21]^
												USARTzTxBuffer[22]^USARTzTxBuffer[23]^USARTzTxBuffer[24]^USARTzTxBuffer[25];
	
	
	com_x_usart_dma_start_tx(27);	//数据包发送
}
//数据接收分析
int8_t get_data_analyze(uint8_t	*pdata)
{
	
	int8_t ret=0;
	int8_t	crc = 0;
	int8_t  p_crc = 0;
	if((*(pdata + 0) == 0xff) && (*(pdata + 1) == 0xff)){
		crc = (*(pdata + 2))^(*(pdata + 3))^(*(pdata + 4))^(*(pdata + 5))^(*(pdata + 6))^(*(pdata + 7))^(*(pdata + 8))^(*(pdata + 9))^(*(pdata + 10))^(*(pdata + 11))^(*(pdata + 12))^(*(pdata + 13));
		p_crc = (int8_t)(*(pdata + 14));//不进行类型转换，负数不正常
	}
	else{
		ret = -1;
		return ret;
	}
	if(p_crc != crc ){//校验和分析有误
		ret = -1;
		return ret;
	}
	//数据包分析正确，提取数据
	memset(&com_x_rcv_data,0,sizeof(com_x_rcv_data));
	com_x_rcv_data.linear_vx.cv[0] = *(pdata + 2);
	com_x_rcv_data.linear_vx.cv[1] = *(pdata + 3);
	com_x_rcv_data.linear_vx.cv[2] = *(pdata + 4);
	com_x_rcv_data.linear_vx.cv[3] = *(pdata + 5);
	
	com_x_rcv_data.linear_vy.cv[0] = *(pdata + 6);
	com_x_rcv_data.linear_vy.cv[1] = *(pdata + 7);
	com_x_rcv_data.linear_vy.cv[2] = *(pdata + 8);
	com_x_rcv_data.linear_vy.cv[3] = *(pdata + 9);
	
	com_x_rcv_data.angular_v.cv[0] = *(pdata + 10);
	com_x_rcv_data.angular_v.cv[1] = *(pdata + 11);
	com_x_rcv_data.angular_v.cv[2] = *(pdata + 12);
	com_x_rcv_data.angular_v.cv[3] = *(pdata + 13);
	
	return ret;
	
}
