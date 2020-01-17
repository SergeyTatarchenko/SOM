/*************************************************
* File Name          : hw_support.c
* Author             : Tatarchenko S.
* Version            : v 1.1
* Description        : hardware support for obj model 
*************************************************/
#include "hw_support.h"
/*-----------------------------------------------*/
#ifdef USE_HWOBJ
/*obj model input driver, get pointer to inputs register array */
void obj_input_driver(uint8_t *registr,int num_of_inputs,int reg_size,OBJ_HW input)
{
	uint8_t *mem_pointer = registr;
	uint8_t reg = *mem_pointer;
		for(int i = 0;i<num_of_inputs;i++)
	{
		for(int j = input;j<reg_size;j++)
		{
//			obj_hw_state(HW_OBJ[j],(reg&input_bit_mask));
			reg>>=input_bit_mask;	
		}
		mem_pointer ++;
		reg = *mem_pointer;
		input += reg_size;
	}
}
#endif

#ifdef USE_HWOBJ
/*----------------------------------------------------------------------
obj value driver from memory to value field
----------------------------------------------------------------------*/
void hw_obj_value_driver(uint16_t *data_pointer,int offset,int size)
{	int i = 0;
	for(i = 0; i <size; i ++)
	{
		if(OBJ_MODEL_CLASS.HW_OBJ[offset + i]->obj_upd_value == TRUE)
		{
			obj_hw_value(OBJ_MODEL_CLASS.HW_OBJ[offset + i],data_pointer[i]);
		}
	}
}
#endif
/*----------------------------------------------------------------------
obj model hw snap state to state, sync when en bit enable in status
----------------------------------------------------------------------*/
void obj_hw_state(OBJ_STRUCT_TypeDef *obj,uint8_t input)
{
	uint8_t id = obj->OBJ_ID.object_id;
	if(input != obj->obj_state)
	{
		obj->obj_state = input;
		FORCED_HANDLER_CALL(id);
	}
}
/*----------------------------------------------------------------------
obj model hw snap value to value,use default value field 
----------------------------------------------------------------------*/
void obj_hw_value(OBJ_STRUCT_TypeDef *obj,uint16_t value)
{
	uint8_t id = obj->OBJ_ID.object_id;
	if(value != obj->OBJ_VALUE.def.default_value)
	{
		obj->OBJ_VALUE.def.default_value = value;
		FORCED_HANDLER_CALL(id);
	}
}


