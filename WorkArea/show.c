#include "show.h"
/**************************************************************************
作者：Mini Balance 
淘宝店铺：http://shop114407458.taobao.com/
**************************************************************************/
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
作    者：Mini Balance
**************************************************************************/

extern int Encoder_A,Encoder_B,Encoder_C;//编码器的脉冲
extern long int Motor_A,Motor_B,Motor_C; //电机的速度
extern MotorControlType MotorControlDEF;	//三轮底盘运动模型分析

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
		//=============刷新=======================//
	  delay_ms(5);
		OLED_Refresh_Gram();	
}
