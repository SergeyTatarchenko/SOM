/*************************************************
* File Name          : hw_support.h
* Author             : Tatarchenko S.
* Version            : v 1.0
* Description        : header for hw_support.c 
*************************************************/
#ifndef HW_SUPPORT_H
#define HW_SUPPORT_H

#include "OBJ_MODEL.h"

/*-----------------------------------------------*/
#if REVISION == 1
	#define NUM_OF_OBJ_ADC	6
	#define NUM_OF_OBJ_INPUTS	8
	#define NUM_OF_OBJ_OUTPUTS	8
	#define NUM_OF_PWM	2
#else
	#error "revision not defined"
#endif

#define in_offset	1
#define out_offset	(NUM_OF_OBJ_INPUTS+1)
#define adc_offset	(out_offset + NUM_OF_OBJ_OUTPUTS+1)
#define pwm_offset	(adc_offset + NUM_OF_OBJ_ADC + 1)

typedef enum
{
	in_0 = in_offset,in_1,in_2,in_3,in_4,in_5,in_6,in_7,
	out_0 = out_offset,out_1,out_2,out_3,out_4,out_5,out_6,out_7,
	adc_0 = adc_offset,adc_1,adc_2,adc_3,adc_4,adc_5,
	pwm_0 = pwm_offset,pwm_1
}obj_hw;

/*-----------------------------------------------*/
void obj_hw_input(OBJ_STRUCT *obj,uint8_t input);
void obj_hw_adc(OBJ_STRUCT *obj,uint16_t value);

void obj_adc_driver(uint16_t* data);
void obj_input_driver(void);
/*-----------------------------------------------*/

/*-----------------------------------------------*/

#endif
