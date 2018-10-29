#include "my_encoder.h"


/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
//	RCC->APB1ENR|=1<<0;     //TIM2时钟使能
//	RCC->APB2ENR|=1<<2;    //使能PORTA时钟
//	GPIOA->CRL&=0XFFFFFF00;//PA0 PA1
//	GPIOA->CRL|=0X00000044;//浮空输入
//	/* 把定时器初始化为编码器模式 */
//	TIM2->PSC = 0x0;//预分频器
//	TIM2->ARR = ENCODER_TIM_PERIOD-1;//设定计数器自动重装值
//	TIM2->CCMR1 |= 1<<0;          //输入模式，IC1FP1映射到TI1上
//	TIM2->CCMR1 |= 1<<8;          //输入模式，IC2FP2映射到TI2上
//	TIM2->CCER |= 0<<1;           //IC1不反向
//	TIM2->CCER |= 0<<5;           //IC2不反向
//	TIM2->SMCR |= 3<<0;	          //SMS='011' 所有的输入均在上升沿和下降沿有效
//	TIM2->CR1 |= 0x01;    //CEN=1，使能定时器
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_Period=ENCODER_TIM_PERIOD-1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//
	TIM_Cmd(TIM2, ENABLE); //
}
/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM4(void)
{
//	RCC->APB1ENR|=1<<2;     //TIM4时钟使能
//	RCC->APB2ENR|=1<<3;     //使能PORTB时钟
//	GPIOB->CRL&=0X00FFFFFF; //PB6 PB7
//	GPIOB->CRL|=0X44000000; //浮空输入
//	/* 把定时器初始化为编码器模式 */
//	TIM4->PSC = 0x0;//预分频器
//	TIM4->ARR = ENCODER_TIM_PERIOD-1;//设定计数器自动重装值
//  TIM4->CCMR1 |= 1<<0;          //输入模式，IC1FP1映射到TI1上
//  TIM4->CCMR1 |= 1<<8;          //输入模式，IC2FP2映射到TI2上
//  TIM4->CCER |= 0<<1;           //IC1不反向
//  TIM4->CCER |= 0<<5;           //IC2不反向
//	TIM4->SMCR |= 3<<0;	          //SMS='011' 所有的输入均在上升沿和下降沿有效
//	TIM4->CR1 |= 0x01;            //CEN=1，使能定时器

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_Period=ENCODER_TIM_PERIOD-1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//
	TIM_Cmd(TIM4, ENABLE); //
}
/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;
   switch(TIMX)
	 {
	   case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;
		 default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}


