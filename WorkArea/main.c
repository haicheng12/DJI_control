#include "sys.h"
#include "control.h"
#include "remote.h"

u8 Flag_Stop=1;//ֹͣ��־λ
int Encoder_A,Encoder_B,Encoder_C;//������������
long int Motor_A,Motor_B,Motor_C; //������ٶ�

int main(void)
{
	Stm32_Clock_Init(9);            //ϵͳʱ������
	SystemInit();                   //=====ϵͳ��ʼ��
	delay_init(72);                 //��ʱ��ʼ��
	
	RC_Init();                      //��ң������ʼ��
	TIM7_Configuration();           //TIM7��ʼ��
	NVIC_Configuration();						//���ȼ�����
	
	CAN1_Configuration();           //CAN1��ʼ��
	//uart_init(72,115200);           //��ʼ������1
	MiniBalance_PWM_Init(7199,0);   //��ʼ��PWM 10KHZ ��Ƶ���Է�ֹ�����Ƶʱ�ļ����
	Encoder_Init_TIM3();            //��ʼ�������� 
	Encoder_Init_TIM4();            //=====��ʼ��������2
	Encoder_Init_TIM5();            //��ʼ��������2 
  Timer2_Init(99,7199);           //10MS��һ���жϷ��������жϷ�������control.c
	OLED_Init();                    //=====OLED��ʼ��

	while(1)
	{	  
		
		  Chassis_Algorithm_Model();		//���̵��ģ��
		
		  TIM7_IRQHandler();
		
		  //PS_Checkout();                //����ŷ�
		  
		  oled_show();             //===��ʾ����  
		
	}
	
}




