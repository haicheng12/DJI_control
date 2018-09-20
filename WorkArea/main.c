#include "sys.h"
#include "control.h"
#include "remote.h"

u8 Flag_Stop=1;//停止标志位
int Encoder_A,Encoder_B,Encoder_C;//编码器的脉冲
long int Motor_A,Motor_B,Motor_C; //电机的速度

int main(void)
{
	Stm32_Clock_Init(9);            //系统时钟设置
	SystemInit();                   //=====系统初始化
	delay_init(72);                 //延时初始化
	
	RC_Init();                      //大疆遥控器初始化
	TIM7_Configuration();           //TIM7初始化
	NVIC_Configuration();						//优先级配置
	
	CAN1_Configuration();           //CAN1初始化
	//uart_init(72,115200);           //初始化串口1
	MiniBalance_PWM_Init(7199,0);   //初始化PWM 10KHZ 高频可以防止电机低频时的尖叫声
	Encoder_Init_TIM3();            //初始化编码器 
	Encoder_Init_TIM4();            //=====初始化编码器2
	Encoder_Init_TIM5();            //初始化编码器2 
  Timer2_Init(99,7199);           //10MS进一次中断服务函数，中断服务函数在control.c
	OLED_Init();                    //=====OLED初始化

	while(1)
	{	  
		
		  Chassis_Algorithm_Model();		//底盘电机模型
		
		  TIM7_IRQHandler();
		
		  //PS_Checkout();                //光电伺服
		  
		  oled_show();             //===显示屏打开  
		
	}
	
}




