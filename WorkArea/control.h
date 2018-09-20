#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265

#define PS_NORMOL_MODE 0X00
#define PS_SMALL_LARGE_MODE 0X10
#define PS_LARGE_SMALL_MODE 0X11

#define OK 0x08

int TIM1_UP_IRQHandler(void);

typedef struct
{
  s16 Vx;																						//Vx�ٶ�
	s16 Vy;																						//Vy�ٶ�
  s16 Vspin;																				//�����ٶ�

	s16 MotorSpeedOne;																//���1�����ٶ�
	s16 MotorSpeedTwo;																//���2�����ٶ�
	s16 MotorSpeedThr;																//���3�����ٶ�
	
}MotorControlType;

typedef struct
{
	
	float X[10];
	float Y[10];
	float spin[10];
	
}PS_CHECK_TYPE;

typedef struct
{
		float Left_control;
		float Left_last;
		u8 Left_PSnum;

		float Down_control;
		float Down_last;
		u8 Down_PSnum;
	
		u8 PS_C_Statu;
}PS_C_TYPE;

void Chassis_Algorithm_Model(void);
void PS_DATA_analyze(u16 PS_data,float *control_data,u8 *PS_num,float *control_last,u8 MODE);				//������ݽ�������
void PS_Checkout(void);																//���λ�ý���
void PS_control_clear(PS_C_TYPE *data);								//���������

void Set_Pwm_A(int moto_a);
void Set_Pwm_B(int moto_b);
void Set_Pwm_C(int moto_c);
void Xianfu_Pwm_A(void);
void Xianfu_Pwm_B(void);
void Xianfu_Pwm_C(void);
int myabs(int a);
int Incremental_PI_A (int Encoder,int Target);
int Incremental_PI_B (int Encoder,int Target);
int Incremental_PI_C (int Encoder,int Target);
#endif
