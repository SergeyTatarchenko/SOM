# SOM
 Object model programming with FreeRTOS support
 
v 0.7
structure:
[header file describing model objects] -> [model configuration header file]->[model description header file]
                                          [memory usage configuration file] 

1) create memory_config.h and model_config.h files from templates and add to project
1) create object name  example   " #define obj_name	(IND_obj_NULL + x)"
2) create object init like "obj_index,class,type,snap,delay,handler in  _obj_cofig_ define	

	index   - [object number in memory];
	class   - [object priority];
	type    - [Object type (obj_soft default)];
	snap    - [hardware binding to the HWOBJ_Event handler (NULL default)];
	delay   - [handler call delay (obj_timer ONLY!!!, (NULL default))];
	handler - [object handler Dummy_Handler(empty handler) default];
			
3) add 	object init to 	_obj_cofig_
4) create obj_model_config.h"

define 	SOM_MODE (APP_MODE or BOOT_MODE)
define variables:
	USE_RTOS
	USE_SERIAL_PORT
	USE_CAN_BUS
	USE_HWOBJ
	USE_TIMERS
	TARGET