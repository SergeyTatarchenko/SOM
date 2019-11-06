# SOM
 Object model programming based on FreeRTOS
v 0.5
1) create object name  example   " #define obj_name	(IND_obj_NULL + x)"
2) create object init like "obj_name_init  obj_index,class,type,snap,delay,handler

	index   - [object number in memory];
	class   - [object priority];
	type    - [Object type (obj_soft default)];
	snap    - (hardware binding to the HWOBJ_Event handler (NULL default));
	delay   - (handler call delay (obj_timer ONLY!!!, (NULL default)));
	handler - (object handler Dummy_Handler(empty handler) default);
			
3) add 	object init to 	_obj_cofig_
4) create obj_model_config.h"
define 	SOM_MODE (APP_MODE or BOOT_MODE)
define variables with TRUE of FALSE:
	RTOS_USAGE
	DEBUG_MODE
	USART_COM_ENABLE	
	CAN_COM_ENABLE
	USART_DATA_FAST
	HARDWARE_OBJECT
	OBJECT_TIMER
define 
	NUM_OF_OBJ_ADC
	NUM_OF_OBJ_INPUTS
	NUM_OF_OBJ_OUTPUTS
	NUM_OF_PWM
	REVISION
	num_of_all_obj
	MES_BUF_SIZE
	NUM_OF_HWOBJ
	NUM_OF_TIMER
	TARGET
