#ifndef __ENCODER_H
#define __ENCODER_H
#include "cmsis_device.h"

#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为F103的定时器是16位的。
void Encoder_Init_TIM2(void);
void Encoder_Init_TIM4(void);
int Read_Encoder(uint8_t TIMX);
#endif
