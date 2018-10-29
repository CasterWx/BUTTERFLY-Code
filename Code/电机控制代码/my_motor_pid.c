#include "my_motor_pid.h"

volatile int Target_velocity=0;  //�趨�ٶȿ��Ƶ�Ŀ���ٶ�Ϊ50������ÿ10ms

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target,yanshi_count,yanshi_flag;

int EXTI9_5_IRQHandler(void)
{
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5)==0)
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		  Flag_Target=!Flag_Target;
		  if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ�����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
			{
			  Get_Angle();                                                        //===������̬
			  return 0;
			}                                                                   //10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
			Encoder_Left=-Read_Encoder(2);                                      //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����Զ�����һ��ȡ������֤�������һ��
			Encoder_Right=Read_Encoder(4);                                      //===��ȡ��������ֵ
	  	Get_Angle();                                                        //===������̬	                                                            //===ɨ�谴��״̬ ����˫�����Ըı�С������״̬
 			Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                   //===ƽ��PID����
		  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
 	    Moto1=Balance_Pwm+Velocity_Pwm;                                     //===�������ֵ������PWM
 	  	Moto2=Balance_Pwm+Velocity_Pwm;                                     //===�������ֵ������PWM
   		Xianfu_Pwm();                                                       //===PWM�޷�
      if(Turn_Off(Angle_Balance)==0)                                      //===����������쳣
 			Set_Pwm(Moto1,Moto2);                                               //===��ֵ��PWM�Ĵ���
	}
	 return 0;
}




/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
			int siqu=400;
			if(moto1>0)
			{
				GPIO_SetBits(GPIOB,GPIO_Pin_15);
				GPIO_ResetBits(GPIOB,GPIO_Pin_14);

//				GPIO_SetBits(GPIOB,GPIO_Pin_12);
//				GPIO_ResetBits(GPIOB,GPIO_Pin_13);
			}
			else
			{
				GPIO_ResetBits(GPIOB,GPIO_Pin_15);
				GPIO_SetBits(GPIOB,GPIO_Pin_14);

//				GPIO_ResetBits(GPIOB,GPIO_Pin_12);
//				GPIO_SetBits(GPIOB,GPIO_Pin_13);
			}
			//PWMA=myabs(moto1);
			TIM_SetCompare1(TIM1,myabs(moto1)+siqu);
			if(moto2>0)
			{
				GPIO_SetBits(GPIOB,GPIO_Pin_12);
				GPIO_ResetBits(GPIOB,GPIO_Pin_13);
			}
			else
			{
				GPIO_ResetBits(GPIOB,GPIO_Pin_12);
				GPIO_SetBits(GPIOB,GPIO_Pin_13);
			}
			TIM_SetCompare4(TIM1,myabs(moto2)+siqu);
}

/**************************************************************************
�������ܣ�����PWM��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{
	  int Amplitude=6900;    //===PWM������7200 ������6900
    if(Moto1<-Amplitude) Moto1=-Amplitude;
		if(Moto1>Amplitude)  Moto1=Amplitude;
	  if(Moto2<-Amplitude) Moto2=-Amplitude;
		if(Moto2>Amplitude)  Moto2=Amplitude;
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
int Incremental_PI (int Encoder,int Target)
{
   float Kp=100,Ki=100;
   static int Bias,Pwm,Last_bias;
   Bias=Encoder-Target;                //����ƫ��
   Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
   Last_bias=Bias;	                   //������һ��ƫ��
   return Pwm;                         //�������
}


int Incremental_PI1 (int Encoder,int Target)
{
   float Kp=100,Ki=100;
   static int Bias,Pwm,Last_bias;
   Bias=Encoder-Target;                //����ƫ��
   Pwm+=Kp*(Bias-Last_bias)+Ki*Bias;   //����ʽPI������
   Last_bias=Bias;	                   //������һ��ƫ��
   return Pwm;                         //�������
}



/**************************************************************************
�������ܣ���ȡ�Ƕ� �����㷨�������ǵĵ�У�����ǳ�����
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
void Get_Angle(void)
{
	Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
	Angle_Balance=Pitch;             //===����ƽ�����
	Gyro_Balance=gyro[1];            //===����ƽ����ٶ�
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(float angle)
{
	    u8 temp;
			if(angle<-40||angle>40)
			{	                                                 //===��Ǵ���40�ȹرյ��
      temp=1;                                            //===Flag_Stop��1�رյ��
//			AIN1=0;                                            //===���������������¶ȹ���ʱ�رյ��
//			AIN2=0;
//			BIN1=0;
//			BIN2=0;
      GPIO_ResetBits(GPIOB,GPIO_Pin_12);
      GPIO_ResetBits(GPIOB,GPIO_Pin_13);
      GPIO_ResetBits(GPIOB,GPIO_Pin_14);
      GPIO_ResetBits(GPIOB,GPIO_Pin_15);
      }
			else
      temp=0;
      return temp;
}

/**************************************************************************
�������ܣ��ٶ�PI���� �޸�ǰ�������ٶȣ�����Target_Velocity�����磬�ĳ�60�ͱȽ�����
��ڲ��������ֱ����������ֱ�����
����  ֵ���ٶȿ���PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{
    static float Velocity,Encoder_Least,Encoder,Movement=0;
	  static float Encoder_Integral;
	  float kp=50,ki=kp/200;
	  //float kp=75,ki=kp/200;//�ظ��ز���
	  //=============�ٶ�PI������=======================//
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩
		Encoder *= 0.7;		                                                //===һ�׵�ͨ�˲���
		Encoder += Encoder_Least*0.3;	                                    //===һ�׵�ͨ�˲���
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Encoder_Integral=Encoder_Integral-Movement_X;                       //===����ң�������ݣ�����ǰ������
		if(Encoder_Integral>15000)  	Encoder_Integral=15000;             //===�����޷�
		if(Encoder_Integral<-15000) 	Encoder_Integral=-15000;            //===�����޷�
		Velocity=Encoder*kp+Encoder_Integral*ki;                          //===�ٶȿ���
		return Velocity;
}

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance(float Angle,float Gyro)
{
   float Bias,kp=100,kd=0.40;
   //float Bias,kp=300,kd=1.0;//�ظ��ز���
	 int balance;
	 Bias=Angle-ZHONGZHI;       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=kp*Bias+Gyro*kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ��
	 return balance;
}


