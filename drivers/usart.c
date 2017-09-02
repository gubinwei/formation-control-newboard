/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：匿名科创
 * 文件名  ：usart.c
 * 描述    ：串口驱动
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
**********************************************************************************/

#include "usart.h"
#include "data_transfer.h"
#include "ultrasonic.h"
#include "led.h"
u8 Rx_2_Buf[256];	
u8 Tx2Buffer[256];
u8 Tx2Counter=0;
u8 count2=0; 
u8 Tx2DMABuffer[256];
void Usart2_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //??USART2??
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	//???????
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);
	
	//??PD5??USART2 Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//??PD6??USART2 Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//??USART2
	//??????
	USART_InitStructure.USART_BaudRate = br_num;       //????????????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8???
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //??????1????
	USART_InitStructure.USART_Parity = USART_Parity_No;    //??????
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //???????
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //???????
	//??USART2??
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //???????
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK??????????->???
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //?????????????
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //?????????????SCLK??
	
	USART_Init(USART2, &USART_InitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStruct);

	//??USART2????
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	//??USART2
	USART_Cmd(USART2, ENABLE); 
	USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
//	//????(????)??
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}
DMA_DeInit(DMA1_Stream6);
	
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}//µÈ´ýDMA¿ÉÅäÖÃ 
	
  /* ÅäÖÃ DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //Í¨µÀÑ¡Ôñ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART2->DR;//DMAÍâÉèµØÖ·
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Tx2DMABuffer;//DMA ´æ´¢Æ÷0µØÖ·
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//´æ´¢Æ÷µ½ÍâÉèÄ£Ê½
  DMA_InitStructure.DMA_BufferSize = 0;//Êý¾Ý´«ÊäÁ¿ 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//ÍâÉè·ÇÔöÁ¿Ä£Ê½
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//´æ´¢Æ÷ÔöÁ¿Ä£Ê½
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//ÍâÉèÊý¾Ý³¤¶È:8Î»
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//´æ´¢Æ÷Êý¾Ý³¤¶È:8Î»
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// Ê¹ÓÃÆÕÍ¨Ä£Ê½ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//ÖÐµÈÓÅÏÈ¼¶
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//´æ´¢Æ÷Í»·¢µ¥´Î´«Êä
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//ÍâÉèÍ»·¢µ¥´Î´«Êä
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);//³õÊ¼»¯DMA Stream
	//DMA_Cmd(DMA1_Stream6, ENABLE);  
	DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);
}


void Usart2_IRQ(void)
{
	u8 com_data;
	
	if(USART2->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART2->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART2,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);//清除中断标志

		com_data = USART2->DR;
		ANO_DT_Data_Receive_Prepare(com_data);
	}
	//发送（进入移位）中断
//	if( USART_GetITStatus(USART2,USART_IT_TC ) )
//	{
//		USART2->DR = TxBuffer[TxCounter++]; //写DR清除中断标志          
//		if(TxCounter == count)
//		{
//			USART2->CR1 &= ~USART_CR1_TCIE;		//关闭TXE（发送中断）中断
//		}


//		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
//	}



}

void Usart2_Send(unsigned char *DataToSend ,u8 data_num)
{
  u8 i;
	static uint16_t num=0;
	static u8 len=0;

	u8 temp;
	DMA_Cmd(DMA1_Stream6, DISABLE);
	DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
	num = DMA_GetCurrDataCounter(DMA1_Stream6);
	
	for(i=0;i<(u8)data_num;i++)
	{
		Tx2Buffer[count2++] = *(DataToSend+i);
	}
	for (i=0;i<(u8)num;i++)
	{
			Tx2DMABuffer[i]=Tx2Buffer[((u8)(len-num+i))];
	}
	for (;i<(u8)(num+data_num);i++)
	{
		Tx2DMABuffer[i]=*(DataToSend+i-num);
	}
	len=count2;
	while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}	//È·±£DMA¿ÉÒÔ±»ÉèÖÃ  
	DMA1_Stream6->NDTR = (uint16_t)(num+data_num);          //Êý¾Ý´«ÊäÁ¿  
	DMA_Cmd(DMA1_Stream6, ENABLE);       


}



void Uart5_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	//USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //开启USART5时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
	
	//配置PC12作为UART5　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//配置PD2作为UART5　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置UART5
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	USART_Init(UART5, &USART_InitStructure);
	

	//使能UART5接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	//使能USART5
	USART_Cmd(UART5, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}

}
u8 Tx5Buffer[256];
u8 Tx5Counter=0;
u8 count5=0; 

void Uart5_IRQ(void)
{
	u8 com_data;
  //接收中断
	if( USART_GetITStatus(UART5,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);//清除中断标志

		com_data = UART5->DR;
		Ultra_Get(com_data);
	}

	//发送（进入移位）中断
	if( USART_GetITStatus(UART5,USART_IT_TXE ) )
	{
				
		UART5->DR = Tx5Buffer[Tx5Counter++]; //写DR清除中断标志
          
		if(Tx5Counter == count5)
		{
			UART5->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}

		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}

}

void Uart5_Send(unsigned char *DataToSend ,u8 data_num)
{
	u8 i;
	for(i=0;i<data_num;i++)
	{
		Tx5Buffer[count5++] = *(DataToSend+i);
	}

	if(!(UART5->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(UART5, USART_IT_TXE, ENABLE); //打开发送中断
	}

}

/*******´®¿Ú1±äÁ¿*********************/
u8 Rx_1_Buf[256];	
u8 Tx1Buffer[256];
u8 Tx1DMABuffer[256];
u8 Tx1Counter=0;
u8 count1=0; 
/***********************************/

void Usart1_Init(u32 br_num)
{ 
 GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); 
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = br_num;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure); 
	
  USART_Cmd(USART1, ENABLE);
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		
	NVIC_Init(&NVIC_InitStructure);	
	
	DMA_DeInit(DMA2_Stream7);
	
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}//µÈ´ýDMA¿ÉÅäÖÃ 
	
  /* ÅäÖÃ DMA Stream */
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //Í¨µÀÑ¡Ôñ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&USART1->DR;//DMAÍâÉèµØÖ·
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)Tx1DMABuffer;//DMA ´æ´¢Æ÷0µØÖ·
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//´æ´¢Æ÷µ½ÍâÉèÄ£Ê½
  DMA_InitStructure.DMA_BufferSize = 0;//Êý¾Ý´«ÊäÁ¿ 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//ÍâÉè·ÇÔöÁ¿Ä£Ê½
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//´æ´¢Æ÷ÔöÁ¿Ä£Ê½
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//ÍâÉèÊý¾Ý³¤¶È:8Î»
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//´æ´¢Æ÷Êý¾Ý³¤¶È:8Î»
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// Ê¹ÓÃÆÕÍ¨Ä£Ê½ 
  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//ÖÐµÈÓÅÏÈ¼¶
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//´æ´¢Æ÷Í»·¢µ¥´Î´«Êä
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//ÍâÉèÍ»·¢µ¥´Î´«Êä
  DMA_Init(DMA2_Stream7, &DMA_InitStructure);//³õÊ¼»¯DMA Stream

}

void USART1_IRQHandler(void)
{
	u8 com_data;
	if(USART1->SR & USART_SR_ORE)//ORE中断
	{

		com_data = USART1->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART1,USART_IT_RXNE) )
	{
		
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除中断标志
		com_data = USART1->DR;
		ANO_DT_Data_Receive_Prepare2(com_data);

	}



}
void Usart1_Send(unsigned char *DataToSend ,u8 data_num)
{
   u8 i;
	static uint16_t num=0;
	static u8 len=0;
	
	DMA_Cmd(DMA2_Stream7, DISABLE);
	DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//Çå³ýDMA2_Steam7´«ÊäÍê³É±êÖ¾
	num = DMA_GetCurrDataCounter(DMA2_Stream7);
	
	for(i=0;i<data_num;i++)
	{
		Tx1Buffer[count1++] = *(DataToSend+i);
	}
	for (i=0;i<num;i++)
	{
		Tx1DMABuffer[i]=Tx1Buffer[((u8)(len-num+i))];
	}
	for (;i<num+data_num;i++)
	{
		Tx1DMABuffer[i]=*(DataToSend+i-num);
	}
	len=count1;
	while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	//È·±£DMA¿ÉÒÔ±»ÉèÖÃ  
	DMA2_Stream7->NDTR = (uint16_t)(num+data_num);          //Êý¾Ý´«ÊäÁ¿  
	DMA_Cmd(DMA2_Stream7, ENABLE);      
}



void Usart3_Init(u32 br_num)
{
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //开启USART2时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);	
	
	//串口中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);
	
	//配置PD5作为USART2　Tx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	//配置PD6作为USART2　Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 
	
	//配置USART2
	//中断被屏蔽了
	USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
	//配置USART2时钟
	USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
	USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
	USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
	USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出
	
	USART_Init(USART3, &USART_InitStructure);
	USART_ClockInit(USART3, &USART_ClockInitStruct);

	//使能USART2接收中断
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能USART2
	USART_Cmd(USART3, ENABLE); 
//	//使能发送（进入移位）中断
//	if(!(USART2->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
//	}


}

u8 TxBuffer3[256];
u8 TxCounter3=0;
u8 count3=0; 

u8 Rx_Buf3[256];	//串口接收缓存

void Usart3_IRQ(void)
{
	u8 com_data;
	
	if(USART3->SR & USART_SR_ORE)//ORE中断
	{
		com_data = USART3->DR;
	}

  //接收中断
	if( USART_GetITStatus(USART3,USART_IT_RXNE) )
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志

		com_data = USART3->DR;
	//	ANO_DT_Data_Receive_Prepare3(com_data);
	}
	//发送（进入移位）中断
	if( USART_GetITStatus(USART3,USART_IT_TXE ) )
	{
		USART3->DR = TxBuffer3[TxCounter3++]; //写DR清除中断标志          
		if(TxCounter3 == count3)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
		}


		//USART_ClearITPendingBit(USART2,USART_IT_TXE);
	}



}

void Usart3_Send(unsigned char *DataToSend ,u8 data_num)
{
  u8 i;
	for(i=0;i<data_num;i++)
	{
		TxBuffer3[count3++] = *(DataToSend+i);
	}

	if(!(USART3->CR1 & USART_CR1_TXEIE))
	{
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //打开发送中断
	}

}

