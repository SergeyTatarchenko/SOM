# SOM
 Object model programming based on FreeRTOS
v 0.4
1) create object name  example   " #define obj_name	(IND_obj_NULL + x)"
2) create object init like "obj_name_init  obj_index,class,type,snap,delay,handler

	index   - [object number in memory];
	class   - [object priority];
	type    - [Object type (obj_soft default)];
	snap    - (hardware binding to the HWOBJ_Event handler (NULL default));
	delay   - (handler call delay (obj_timer ONLY!!!, (NULL default)));
	handler - (object handler Dummy_Handler(empty handler) default);
			
3) add 	object init to 	_obj_cofig_
