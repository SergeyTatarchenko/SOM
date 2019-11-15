/*************************************************
* File Name          : hw_support.c
* Author             : Tatarchenko S.
* Version            : v 1.0
* Description        : hardware support for obj model 
*************************************************/
#include "hw_support.h"
/*-----------------------------------------------*/
#if HARDWARE_OBJECT == TRUE
/*obj model ADC driver, get pointer to adc data array */
void obj_adc_driver(uint16_t* data)
{
	int j = 0;
	for(int i = adc_0;i < (adc_0 + NUM_OF_OBJ_ADC);i++)
	{
		obj_hw_value(HW_OBJ[i],data[j]);
		j++;
	}
}
/*obj model input driver, get pointer to inputs register array */
void obj_input_driver(uint8_t *registr,int num_of_inputs,int reg_size,OBJ_HW input)
{
	uint8_t *mem_pointer = registr;
	uint8_t reg = *mem_pointer;
		for(int i = 0;i<num_of_inputs;i++)
	{
		for(int j = input;j<reg_size;j++)
		{
			obj_hw_state(HW_OBJ[j],(reg&input_bit_mask));
			reg>>=input_bit_mask;	
		}
		mem_pointer ++;
		reg = *mem_pointer;
		input += reg_size;
	}
}
#endif

/*obj model hw snap state to state */
void obj_hw_state(OBJ_STRUCT *obj,uint8_t input)
{
	if(obj->obj_hardware){
		if(input != obj->obj_state)
		{
			/*event to SOM*/
			if(!obj->obj_event)
			{
				obj->obj_state = input;
			}
			obj_update(obj->idof_obj);
		}
	}
}

/*obj model hw snap value to value */
void obj_hw_value(OBJ_STRUCT *obj,uint16_t value)
{
	if(obj->obj_hardware)
	{
		/* value update */
		if(value != obj->obj_value)
		{
			obj->obj_value = value;
			obj_update(obj->idof_obj);
		}
	}
}


