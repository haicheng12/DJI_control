#include "show.h"
/**************************************************************************
���ߣ�Mini Balance 
�Ա����̣�http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
��    �ߣ�Mini Balance
**************************************************************************/

extern int Encoder_A,Encoder_B,Encoder_C;//������������
extern long int Motor_A,Motor_B,Motor_C; //������ٶ�
extern MotorControlType MotorControlDEF;	//���ֵ����˶�ģ�ͷ���

void oled_show(void)
{
	  OLED_ShowString(0,00,"Three_omni_wheel");
	  OLED_ShowString(0,10,"Angle???");
//		OLED_ShowNumber(20,10,MotorControlDEF.Vx,3,12);
	
//	  OLED_ShowString(0,20,"Y:");
//		OLED_ShowNumber(20,20,MotorControlDEF.Vy,3,12);
//	
//	  OLED_ShowString(0,30,"Z:");
//		OLED_ShowNumber(20,30,MotorControlDEF.Vspin,3,12);
		//=============ˢ��=======================//
	  delay_ms(5);
		OLED_Refresh_Gram();	
}
