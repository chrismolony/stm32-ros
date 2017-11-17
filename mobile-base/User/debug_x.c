
#include "debug_x.h"
#include "stm32f10x.h"
#include "string.h"
#include "stdio.h"
/*****************************************************
@StevenShi
STM32 USART1 DMA驱动，仅用于debug调试用
重定向printf函数到DMA
在需要打印信息的地方直接调用printf函数即可
没有用到接收功能，可以关闭该功能
*****************************************************/
USARTm_Tx_Buffer_TypeDef USARTm_Tx_Buf_Queue;//队列申请

//数据入列
uint32_t debug_x_usart_In_Queue(uint8_t data) 
{ 	
	if(USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER <USARTm_Tx_BUFFER_SIZE)//判断队列是否满
		{
			if(USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_HEAD >= USARTm_Tx_BUFFER_SIZE) // 判断队列头指针是否超出队列宽度
				USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_HEAD = 0; //超出后将头指针指向0 		
			USARTm_Tx_Buf_Queue.USARTm_Tx_Buffer[USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_HEAD] = data; //将数据放入队列		
			USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_HEAD++; 	//头指针移动	
			USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER++;  	//当前队列长度加一
			return 0; 	
		} 		
	return 1;	 	 
}
//数据出列
uint32_t debug_x_usart_Out_Queue(uint8_t *data) 
{ 	
	uint32_t num;
	if(USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER > 0) 	//队列不为空才能进行读出操作
		{ 		
			if(USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_TAIL >= USARTm_Tx_BUFFER_SIZE)//判断队列尾指针是否超出队列宽度 			
				USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_TAIL = 0;//超出后置0
			num = USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER;//记录当前队列数据长度		
			*data = USARTm_Tx_Buf_Queue.USARTm_Tx_Buffer[USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_TAIL]; //从队列中读出一个数		
			USARTm_Tx_Buf_Queue.USARTm_Tx_PTR_TAIL++; //尾指针前移		
			USARTm_Tx_Buf_Queue.USARTm_Tx_COUNTER--; //队列宽度减一 		
			return num; //返回队列数据长度
		} 	
	else 	
		{ 		
			*data = 0xFF; //队列为空，填充一个值		
			return 0;//返回队列长度0 	
		} 
}


uint8_t USARTmTxBuffer[USARTm_Tx_BUFFER_SIZE];//发送缓存
uint8_t USARTmRxBuffer[USARTm_Rx_BUFFER_SIZE];//接收缓存
uint8_t USARTmRxBufferD[USARTm_Rx_BUFFER_SIZE];//接收缓存




//串口参数配置-9600 8bit 1stop even parity
void debug_x_usart_config(void)
{
	// USARTm and USARTm configuration
	/* Configure USARTm */
	USART_InitTypeDef USARTm_InitStructure;
	USARTm_InitStructure.USART_BaudRate = 9600;
	USARTm_InitStructure.USART_WordLength = USART_WordLength_9b;//系统bug USART_WordLength_8b无法正常发送
	USARTm_InitStructure.USART_StopBits = USART_StopBits_1;
	USARTm_InitStructure.USART_Parity = USART_Parity_Even;
	USARTm_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USARTm_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USARTm, &USARTm_InitStructure);

	/* Enable USARTm Receive  interrupts */
	USART_ITConfig(USARTm, USART_IT_IDLE, ENABLE); //开启串口空闲IDLE中断
	/* Enable USARTm DMA TX request */
	USART_DMACmd(USARTm, USART_DMAReq_Tx, ENABLE);
	/* Enable USARTm DMA RX request */
	USART_DMACmd(USARTm, USART_DMAReq_Rx, ENABLE);
	/* Enable USARTm */
	USART_Cmd(USARTm, ENABLE);
	/* Enable USARTm DMA TX Channel */
	USART_DMACmd(USARTm, USART_DMAReq_Tx,ENABLE);// DMA_Cmd(USARTz_Tx_DMA_Channe, ENABLE);

	/* Enable USARTm DMA RX Channel */
	USART_DMACmd(USARTm, USART_DMAReq_Rx,ENABLE);//DMA_Cmd(USARTz_Rx_DMA_Channe, ENABLE);
	
}
//GPIO配置
void debug_x_usart_gpio_config(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USARTm Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//
	GPIO_InitStructure.GPIO_Pin = USARTm_RxPin;
	GPIO_Init(USARTm_GPIO, &GPIO_InitStructure);  

	/* Configure USARTm Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用形式的推挽输出
	GPIO_InitStructure.GPIO_Pin = USARTm_TxPin;
	GPIO_Init(USARTm_GPIO, &GPIO_InitStructure);


}



//NVIC配置
void debug_x_uasrt_nvic_config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);

	/* Enable the DMA Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = (uint8_t)UASRTm_TX_DMA_IRQ;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable the USARTm Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USARTm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//DMA-发送通道配置
void debug_x_usart_dma_tx_config(uint32_t memoryBaseAddr, uint8_t sendBufferSize)
{
	DMA_InitTypeDef DMA_InitStructure;

	/* USARTm_Tx_DMA_Channel  */
	DMA_Cmd(USARTm_Tx_DMA_Channe,DISABLE);//stop dma
	DMA_DeInit(USARTm_Tx_DMA_Channe);//恢复缺省配置
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTm_DR_Base;//串口发送数据寄存器
	//DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTmTxBuffer;//发送缓冲首地址
	//DMA_InitStructure.DMA_PeripheralBaseAddr = peripheralBaseAddr;//串口发送数据寄存器
	DMA_InitStructure.DMA_MemoryBaseAddr = memoryBaseAddr;//发送缓冲首地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//设置外设为目标
	DMA_InitStructure.DMA_BufferSize = sendBufferSize;//需要发送的字节数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整       
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整              
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节        
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级                
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式 
	DMA_Init(USARTm_Tx_DMA_Channe, &DMA_InitStructure);//写入配置	
	DMA_ClearFlag(USARTm_Tx_DMA_FLAG);    //清除DMA所有标志                          
	//DMA_Cmd(USARTm_Tx_DMA_Channe, ENABLE); 
	DMA_ITConfig(USARTm_Tx_DMA_Channe, DMA_IT_TC, ENABLE);  //开启DMA发送通道中断  
}
//DMA-接收通道配置
void debug_x_usart_dma_rx_config(void)	
{
	DMA_InitTypeDef DMA_InitStructure;

	/*USARTm_Rx_DMA_Channe*/
	DMA_Cmd(USARTm_Rx_DMA_Channe,DISABLE);//stop dma
	DMA_DeInit(USARTm_Rx_DMA_Channe);//恢复缺省配置
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTm_DR_Base;//设置串口接收数据寄存器
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USARTmRxBuffer;//接收缓存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//设置外设为数据源
	DMA_InitStructure.DMA_BufferSize = USARTm_Tx_BUFFER_SIZE;//需要接收的字节数
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址不做增加调整       
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存缓冲区地址增加调整              
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //外设数据宽度 一个字节
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //内存数据宽度一个字节        
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                           //单次传输模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //高优先级                
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //关闭内存到内存的DMA模式 
	DMA_Init(USARTm_Rx_DMA_Channe, &DMA_InitStructure);//写入配置	
	DMA_ClearFlag(USARTm_Rx_DMA_FLAG);    //清除DMA所有标志                          
	DMA_Cmd(USARTm_Rx_DMA_Channe, ENABLE); //开启DMA接收通道
	   
}
//DMA-发送完成中断
void DMA1_Channel4_IRQHandler(void)//中断向量定义在startup_stm32f10x.s文件中
{
	if(DMA_GetITStatus(DMA1_FLAG_TC4))
	{
		DMA_ClearFlag(USARTm_Tx_DMA_FLAG);    //清除DMA所有标志    
		DMA_Cmd(USARTm_Tx_DMA_Channe, DISABLE);  //关闭DMA发送通道
	}
}


//开始DMA发送 memoryBaseAddr-要发送的数据首地址；SendBufferSize-数据长度
void debug_x_usart_dma_start_tx(uint32_t memoryBaseAddr,uint32_t SendBufferSize)
{
	debug_x_usart_dma_tx_config(memoryBaseAddr,SendBufferSize);
	USARTm_Tx_DMA_Channe->CNDTR = (uint16_t)SendBufferSize; //重新赋值 指定发送缓存长度
	DMA_Cmd(USARTm_Tx_DMA_Channe, ENABLE);  //开启DMA发送      
}
//**************************************************************************

//串口空闲中断-一帧完成，将数据读入
void USART1_IRQHandler(void)//中断向量定义在startup_stm32f10x.s中
{
    
	if(USART_GetITStatus(USARTm, USART_IT_IDLE) != RESET)  
		{
				debug_x_usart_dma_read();
				USART_ReceiveData( USARTm );
		}
}
//DMA读出一帧数据，开始下一帧的等待接收
void debug_x_usart_dma_read(void)
{
	uint8_t rxcounter;
	uint8_t i;
	DMA_Cmd(USARTm_Rx_DMA_Channe, DISABLE);    //关闭DMA防止干扰   
	DMA_ClearFlag( USARTm_Rx_DMA_FLAG );   //清除标志位       
	rxcounter= USARTm_Tx_BUFFER_SIZE - DMA_GetCurrDataCounter(USARTm_Rx_DMA_Channe);//获取接收到的字节数 
	USARTm_Rx_DMA_Channe->CNDTR = USARTm_Tx_BUFFER_SIZE; //重新赋值计数值   
	memset(USARTmRxBufferD,0,sizeof(USARTmRxBufferD));
	for(i=0;i<rxcounter;i++){
		USARTmRxBufferD[i] = USARTmRxBuffer[i];//获取接收到的数据，存入数组RxBufferD中
	}

	//debug_x_get_data(USARTmRxBufferD);//分析收到的数据包
	for(i=0;i<rxcounter;i++)
		USARTmRxBuffer[i] = 0;//clear Rx buffer
	DMA_Cmd(USARTm_Rx_DMA_Channe, ENABLE);  //DMA开启 等待下一帧数据     
    
   
   
}
//RCC配置
void debug_x_usart_rcc_config(void)
{
		/* DMA clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_APB2PeriphClockCmd( USARTm_GPIO_CLK  |RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(USARTm_CLK,ENABLE);

}
//外部调用函数 初始化
void debug_x_usart_Init(void)
{
	
	debug_x_usart_rcc_config();
	debug_x_uasrt_nvic_config();
	debug_x_usart_gpio_config();

	debug_x_usart_dma_rx_config();//接收配置
	debug_x_usart_config();
}
//针对DMA的printf实现 每次调用printf 都需要调用一次该函数
//也可以将该函数放到定时中断中去执行 比如sysTick
int debug_x_usart_dma_ctl(void) 
{ 	
	uint32_t num=0; 	uint8_t data; 	
	if(DMA_GetCurrDataCounter(USARTm_Tx_DMA_Channe)==0) 
	{
		DMA_Cmd(USARTm_Tx_DMA_Channe,DISABLE); 		
		while((debug_x_usart_Out_Queue(&data))!=0) 		
		{	 			
			USARTmTxBuffer[num]=data; 			
			num++; 			
			if(num==USARTm_Tx_BUFFER_SIZE) 			
				break; 		
		} 		
		if(num>0) 		
		{ 			
			debug_x_usart_dma_start_tx((uint32_t)USARTmTxBuffer,num);
		} 		
		return 0; 	
	} 	
	else 	
		return 1; 
}
/*********************************用于printf的重定向*****************************************************************/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART */
	//USART_SendData(USART1, (uint8_t) ch);
	/* Loop until the end of transmission */
	// while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	debug_x_usart_In_Queue((uint8_t)ch);//将要发送的数据放入队列
	return ch;
}
