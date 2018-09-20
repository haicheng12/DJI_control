#include "remote.h"
#include "control.h"
volatile unsigned char sbus_rx_buffer[25];
short bb;
RC_Ctl_t test;

extern MotorControlType MotorControlDEF;	//三轮底盘运动模型分析

void RC_Init(void)
{
/* -------------- Enable Module Clock Source ----------------------------*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA  , ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE);
/* -------------- Configure GPIO ---------------------------------------*/
	{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart2;
		gpio.GPIO_Pin = GPIO_Pin_3;
		gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpio);
		USART_DeInit(USART2);
		usart2.USART_BaudRate = 100000;
		usart2.USART_WordLength = USART_WordLength_8b;
		usart2.USART_StopBits = USART_StopBits_1;
		usart2.USART_Parity = USART_Parity_Even;
		usart2.USART_Mode = USART_Mode_Rx;
		usart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2,&usart2);
		USART_Cmd(USART2,ENABLE);
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
		USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	}


	{
		DMA_InitTypeDef dma;
		DMA_DeInit(DMA1_Channel6);
		dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
		dma.DMA_MemoryBaseAddr = (uint32_t)sbus_rx_buffer;
		dma.DMA_DIR = DMA_DIR_PeripheralSRC;
		dma.DMA_BufferSize = 18;
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
		dma.DMA_Mode = DMA_Mode_Circular;
		dma.DMA_Priority = DMA_Priority_VeryHigh;
		dma.DMA_M2M=DMA_M2M_Disable;
		DMA_Init(DMA1_Channel6,&dma);
		DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);
		DMA_Cmd(DMA1_Channel6,ENABLE);
	}
}

void DMA1_Channel6_IRQHandler(void)
{

	if(DMA_GetITStatus( DMA1_IT_TC6))
	{
		DMA_ClearFlag(DMA1_FLAG_TC6);
		DMA_ClearITPendingBit(DMA1_IT_TC6);
		RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
		RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
		RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | //!< Channel 2
		(sbus_rx_buffer[4] << 10)) & 0x07ff;
		RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
		RC_Ctl.rc.s1 = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left
		RC_Ctl.rc.s2 = ((sbus_rx_buffer[5] >> 4)& 0x0003); //!< Switch right
		RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8); //!< Mouse X axis
		RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8); //!< Mouse Y axis
		RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8); //!< Mouse Z axis
		RC_Ctl.mouse.press_l = sbus_rx_buffer[12]; //!< Mouse Left Is Press ?
		RC_Ctl.mouse.press_r = sbus_rx_buffer[13]; //!< Mouse Right Is Press ?
		RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8); //!< KeyBoard value
		test=RC_Ctl;
	}
}

void TIM7_Configuration(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
   
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

  TIM_TimeBaseStructure.TIM_Period = 10000-1;
  TIM_TimeBaseStructure.TIM_Prescaler =72-1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;       
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
  TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
  TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);
   
  TIM_UpdateRequestConfig(TIM7,TIM_UpdateSource_Regular);
  TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM7,ENABLE);

}

void TIM7_IRQHandler(void)
{
	
		if(test.rc.ch2>=364&&test.rc.ch2<1024)
		{
			MotorControlDEF.Vx=(1024-test.rc.ch2)/6;
		}
		
		if(test.rc.ch2>1024&&test.rc.ch2<=1684)
		{
			MotorControlDEF.Vx=(1024-test.rc.ch2)/6;
		}
		
		if(test.rc.ch3>=364&&test.rc.ch3<1024)
		{
			MotorControlDEF.Vy=-(1024-test.rc.ch3)/6;
		}
		
		if(test.rc.ch3>1024&&test.rc.ch3<=1684)
		{
			MotorControlDEF.Vy=-(1024-test.rc.ch3)/6;
		}
		
		if(test.rc.ch0>=364&&test.rc.ch0<1024)
		{
			MotorControlDEF.Vspin=(1024-test.rc.ch0)/6;
		}
		
		if(test.rc.ch0>1024&&test.rc.ch0<=1684)
		{
			MotorControlDEF.Vspin=(1024-test.rc.ch0)/6;
		}
		
		if(test.rc.ch2==1024&&test.rc.ch3==1024&&test.rc.ch0==1024)
		{
			 MotorControlDEF.Vx=0;
			 MotorControlDEF.Vy=0;
			 MotorControlDEF.Vspin=0;
		}
		
		Chassis_Algorithm_Model();
		TIM_ClearITPendingBit(TIM7,TIM_FLAG_Update);
		
}

