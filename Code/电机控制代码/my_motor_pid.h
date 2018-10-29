#ifndef __MY_MOTOR_PID_H
#define __MY_MOTOR_PID_H
#include "cmsis_device.h"
#include "my_pwm.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "mpu6050.h"
#include "my_encoder.h"
#include "my_uart.h"

#define PI 3.14159265
#define ZHONGZHI 1
extern	int Balance_Pwm,Velocity_Pwm;
extern int Encoder_Left,Encoder_Right;                     //左右编码器的脉冲计数
extern int Moto1,Moto2;                                     //电机PWM变量 应是motor的 向moto致敬
extern float Angle_Balance,Gyro_Balance;           //平衡倾角 平衡陀螺仪 转向陀螺仪
int TIM1_UP_IRQHandler(void);
void Set_Pwm(int moto1,int moto2);
void Xianfu_Pwm(void);
int myabs(int a);
int Incremental_PI (int Encoder,int Target);
int Incremental_PI1 (int Encoder,int Target);
u8 Turn_Off(float angle);
int balance(float angle,float gyro);
void Get_Angle(void);
int velocity(int encoder_left,int encoder_right);
extern int Motor1,Motor2;
extern volatile float Movement_X;
#endif
