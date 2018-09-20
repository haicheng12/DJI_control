#include "control.h"	
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
extern PS_TYPE PS_DEF;													//��紫��������
float PS_CV[15]={30,27,25,23,20,17,15,0,-15,-17,-20,-23,-25,-27,-30};

MotorControlType MotorControlDEF;	//���ֵ����˶�ģ�ͷ���
PS_CHECK_TYPE PS_CHECK_DEF; //��紫�����������
PS_C_TYPE PS_C_DEF;//��紫��������
						
int TIM2_IRQHandler(void)  
{    
	if(TIM2->SR&0X0001)//10ms��ʱ�ж�
	{   
		  TIM2->SR&=~(1<<0);                                       //===�����ʱ��1�жϱ�־λ		 
		  Encoder_A=Read_Encoder(3); //PB14 PB15  ������PA6 PA7  PWMA=PA10 //===��ȡ��������ֵ��M�����٣����Ϊÿ10ms��������
		  Encoder_B=Read_Encoder(4); //PB12 PB13  ������PB6 PB7  PWMB=PA11
		  Encoder_C=Read_Encoder(5); //PB10 PB11  ������PA0 PA1  PWMC=PC6 		 
      Motor_A=Incremental_PI_A(Encoder_A,MotorControlDEF.MotorSpeedOne);//===�ٶ�PI������
		  Motor_B=Incremental_PI_B(Encoder_B,MotorControlDEF.MotorSpeedTwo);
		  Motor_C=Incremental_PI_C(Encoder_C,MotorControlDEF.MotorSpeedThr);
	  	Xianfu_Pwm_A();                                            //===PWM�޷�
		  Xianfu_Pwm_B();                                            //===PWM�޷�
		  Xianfu_Pwm_C(); 
    	Set_Pwm_A(Motor_A);                                        //===��ֵ��PWM�Ĵ���  
		  Set_Pwm_B(Motor_B);                                        //===��ֵ��PWM�Ĵ���  
		  Set_Pwm_C(Motor_C);
 
	}       	
	 return 0;	  
} 

void Chassis_Algorithm_Model(void)//���̵���ٶȷ���ģ��
{

//����ȫ�����˶�ģ��
	
	 MotorControlDEF.MotorSpeedOne =   -MotorControlDEF.Vx																				-	MotorControlDEF.Vspin;		
	 MotorControlDEF.MotorSpeedTwo =  MotorControlDEF.Vx*sin(PI/6)+MotorControlDEF.Vy*cos(PI/6)  -  MotorControlDEF.Vspin;
	 MotorControlDEF.MotorSpeedThr =  MotorControlDEF.Vx*sin(PI/6)-MotorControlDEF.Vy*cos(PI/6)  -  MotorControlDEF.Vspin; 
	
}

//PS_data������ԭʼ����
//control_data�����ѭ�ߵĿ�����
//PS_num��������ĸ���

void PS_DATA_analyze(u16 PS_data,float *control_data,u8 *PS_num,float *control_last,u8 MODE)//������ݽ���
{
	u8 cir;
	u8 continuity_count_statu=0,last_cir_num,PS_count=0;
	if(MODE==PS_NORMOL_MODE)				//��������
	{
		for(cir=1;cir<16;cir++)
		{
			if(((PS_data>>cir)&0x1)==1)
			{
				*control_data=*control_data+PS_CV[cir];
				(*PS_num)++;
			}
		}
		if(*PS_num==0)
		{
			*control_data=*control_last;
		}
		else
		{
			*control_data=*control_data/(*PS_num);
		}
	}
	else if(MODE==PS_SMALL_LARGE_MODE)			//��С����ȡ������������������Ϊֵ
	{
		for(cir=1;cir<16;cir++)
		{
			if(((PS_data>>cir)&0x1)==1)
			{
				(*PS_num)++;
				if(continuity_count_statu==0&&PS_count<3)
				{
					PS_count++;
					if(PS_count==1)
					{
							*control_data=*control_data+PS_CV[cir];
					}
					else
					{
							if(cir-last_cir_num==1)
							{
								*control_data=*control_data+PS_CV[cir];
							}
							else
							{
								continuity_count_statu=0x08;
								PS_count--;
							}
					}
					last_cir_num=cir;
				}
			}
			if(continuity_count_statu==0x08)
			{
				if(PS_count==1)
				{
					continuity_count_statu=0;
					PS_count=0;
					*control_data=0;
				}
			}
		}
		if(PS_count==0)
		{
			*control_data=*control_last;
		}
		else
		{
			*control_data=*control_data/PS_count;
		}
	}
	else if(MODE==PS_LARGE_SMALL_MODE)		//�Ӵ�С��ȡ������������������Ϊֵ
	{
		for(cir=15;cir>0;cir--)
		{
			if(((PS_data>>cir)&0x1)==1)
			{
				(*PS_num)++;
				if(continuity_count_statu==0&&PS_count<3)
				{
					PS_count++;
					if(PS_count==1)
					{
							*control_data=*control_data+PS_CV[cir];
					}
					else
					{
							if(last_cir_num-cir==1)
							{
								*control_data=*control_data+PS_CV[cir];
							}
							else
							{
								continuity_count_statu=0x08;
								PS_count--;
							}
					}
					last_cir_num=cir;
				}
			}
			if(continuity_count_statu==0x08)
			{
				if(PS_count==1)
				{
					continuity_count_statu=0;
					PS_count=0;
					*control_data=0;
				}
			}
		}
		if(PS_count==0)
		{
			*control_data=*control_last;
		}
		else
		{
			*control_data=*control_data/PS_count;
		}
	}
}

void PS_Checkout(void)  //���λ�ý���
{
		PS_control_clear(&PS_C_DEF);
		PS_DATA_analyze(PS_DEF.Left.DATA_16,&PS_C_DEF.Left_control,&PS_C_DEF.Left_PSnum,&PS_C_DEF.Left_last,PS_NORMOL_MODE);
		PS_DATA_analyze(PS_DEF.Back.DATA_16,&PS_C_DEF.Down_control,&PS_C_DEF.Down_PSnum,&PS_C_DEF.Down_last,PS_NORMOL_MODE);
		
	  MotorControlDEF.Vx=-PS_C_DEF.Down_control;
	
	  MotorControlDEF.Vy=-PS_C_DEF.Left_control;

		if(PS_C_DEF.Left_control==0&&PS_C_DEF.Down_control==0&&PS_C_DEF.Left_PSnum!=0&&PS_C_DEF.Down_PSnum!=0)
		{
			PS_C_DEF.PS_C_Statu=OK;
		}
	
}

void PS_control_clear(PS_C_TYPE *data)
{	
	data->Left_last=data->Left_control;
	data->Left_control=0;
	data->Left_PSnum=0;
	
	data->Down_last=data->Down_control;
	data->Down_control=0;
	data->Down_PSnum=0;
}

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm_A(int moto_a)
{
			if(moto_a>0)			AIN2=1,			AIN1=0;
			else 	          AIN2=0,			AIN1=1;
			PWMA=myabs(moto_a);
}

void Set_Pwm_B(int moto_b)
{
			if(moto_b>0)			BIN2=1,			BIN1=0;
			else 	          BIN2=0,			BIN1=1;
			PWMB=myabs(moto_b);
}

void Set_Pwm_C(int moto_c)
{
			if(moto_c>0)			CIN2=1,			CIN1=0;
			else 	          CIN2=0,			CIN1=1;
			PWMC=myabs(moto_c);
}

/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm_A(void)
{	
	  int Amplitude=7100;    //===PWM������7200 ������7100
    if(Motor_A<-Amplitude) Motor_A=-Amplitude;	
		if(Motor_A>Amplitude)  Motor_A=Amplitude;	
}

void Xianfu_Pwm_B(void)
{	
	  int Amplitude=7100;    //===PWM������7200 ������7100
    if(Motor_B<-Amplitude) Motor_B=-Amplitude;	
		if(Motor_B>Amplitude)  Motor_B=Amplitude;	
}

void Xianfu_Pwm_C(void)
{	
	  int Amplitude=7100;    //===PWM������7200 ������7100
    if(Motor_C<-Amplitude) Motor_C=-Amplitude;	
		if(Motor_C>Amplitude)  Motor_C=Amplitude;	
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (int Encoder,int Target)
{ 	
   float Kp=20,Ki=30;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}

int Incremental_PI_B (int Encoder,int Target)
{ 	
   float Kp=20,Ki=30;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}

int Incremental_PI_C (int Encoder,int Target)
{ 	
   float Kp=20,Ki=30;	
	 static int Bias,Pwm,Last_bias;
	 Bias=Encoder-Target;                //����ƫ��
	 Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
	 Last_bias=Bias;	                   //������һ��ƫ�� 
	 return Pwm;                         //�������
}

