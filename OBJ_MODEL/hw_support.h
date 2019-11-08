/*************************************************
* File Name          : hw_support.h
* Author             : Tatarchenko S.
* Version            : v 1.1
* Description        : header for hw_support.c 
*************************************************/
#ifndef HW_SUPPORT_H
#define HW_SUPPORT_H
#include "OBJ_MODEL.h"

/*-----------------------------------------------*/

#ifndef REVISION
	#error "revision not defined"
#endif

#ifndef NUM_OF_OBJ_INPUTS
	#error "define number of inputs"
#endif

#ifndef NUM_OF_OBJ_OUTPUTS
	#error "define number of outputs"
#endif

#ifndef NUM_OF_OBJ_ADC
	#error "define number of ADC"
#endif

#define input_bit_mask	0x01	
/*-----------------------------------------------*/
void obj_hw_state(OBJ_STRUCT_TypeDef *obj,uint8_t input);
void obj_hw_value(OBJ_STRUCT_TypeDef *obj,uint16_t value);

void obj_adc_driver(uint16_t* data);
void obj_input_driver(uint8_t *registr,int num_of_inputs,int reg_size,OBJ_HW input);
/*-----------------------------------------------*/

/*-----------------------------------------------*/

#endif
