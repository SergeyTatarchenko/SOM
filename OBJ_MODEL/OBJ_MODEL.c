/*************************************************
* File Name          : OBJ_MODEL.c
* Author             : Tatarchenko S.
* Version            : v 1.6
* Description        : Simple Obj Model 
*************************************************/
#include "OBJ_MODEL.h"
/*----------- global variables-------------------*/

/*structure with current board mode*/
BOARD_STATE	board_state;

/*
	number of objects created;
	the variable must not exceed the value of num_of_all_obj !!!
*/
uint32_t num_of_obj;

#if RTOS_USAGE == TRUE
/*tasks init struct*/
OBJ_MODEL_PRIORITY task_priority;
#endif
#if OBJECT_TIMER == TRUE
	TimerHandle_t obj_timers[NUM_OF_TIMER];
#endif

#ifdef USART_COM_ENABLE
	/* data array for usart obj receive */
	uint8_t usart_data_receive_array[USART1_DEFAULT_BUF_SIZE];
	/*usart data byte counter */
	uint8_t usart_irq_counter = 0;
	#if RTOS_USAGE == TRUE
		/*mutex  to perform currect usart transmit */
		xSemaphoreHandle xMutex_USART_BUSY;
		/*queue of messages from usart module*/
		xQueueHandle usart_receive_buffer;
	#endif
#endif

OBJ_MODEL_CLASS_TypeDef OBJ_MODEL_CLASS;
/*
obj model init function, fill struct fields, snap handlers,fill hardware 
and soft timer objects; 
*/
void obj_model_init()
{
	int i = 0;
	OBJ_INIT_TypeDef _model_init_[] ={_obj_cofig_};
	memset(OBJ_MODEL_CLASS.OBJ_AREA.OBJ_MEMORY_AREA,0,sizeof(OBJ_MODEL_CLASS.OBJ_AREA.OBJ_MEMORY_AREA));
	OBJ_MODEL_CLASS.objDefault = (OBJ_STRUCT_TypeDef*)OBJ_MODEL_CLASS.OBJ_AREA.OBJ_MEMORY_AREA;
	for( i = 0; i <= num_of_all_obj; i++ )
	{
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[i].OBJ_ID.object_id = i;
		OBJ_MODEL_CLASS.OBJ_HANDLERS[i] = (void(*)(void*))Dummy_Handler;
	}
	#ifdef USE_HWOBJ
	for( i = 0; i <= NUM_OF_HWOBJ; i++ )
	{
		OBJ_MODEL_CLASS.HW_OBJ[i] = OBJ_MODEL_CLASS.objDefault;	
	}
	#endif
	#ifdef USE_SERIAL_PORT
	memset(OBJ_MODEL_CLASS.USART_DATA,0,sizeof(USART_FRAME_TypeDef)*num_of_all_obj);
	#endif
	/* object create and handler mapping*/
	obj_bind(_model_init_,sizeof(_model_init_));
	num_of_obj = 0;
	for(int i = 0;i<=num_of_all_obj;i++)
	{
		if(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[i].OBJ_ID.object_class != 0)
		{
			num_of_obj++;
		}
	}
}
/* 
object create and handler mapping function
*/
void obj_bind(OBJ_INIT_TypeDef* _model_init_,int _model_size_)
{
	int i = 0;
	uint8_t obj_quantity = _model_size_/sizeof(OBJ_INIT_TypeDef);
	
	for(i = 0 ; i < obj_quantity ; i++)
	{	
		if(_model_init_[i].obj_type == obj_soft)
		{
		/*default soft obj create*/
		obj_soft_create( _model_init_[i].id,_model_init_[i].obj_class );
		}
		#ifdef USE_HWOBJ
		if(_model_init_[i].obj_type == obj_hard)
		{
		/*hardware obj create, use special weak handler*/
		 obj_hardware_create(_model_init_[i].id,_model_init_[i].obj_class,_model_init_[i].HW_adress);
		}
		#endif
		#ifdef USE_TIMERS
		if(_model_init_[i].obj_type == obj_timer)
		{
		/*hardware obj create, use special weak handler*/
		 obj_timer_create(_model_init_[i].id,_model_init_[i].obj_class,_model_init_[i].delay,_model_init_[i].handler_pointer);
		}
		#endif
		/*handlers swap*/
		if(_model_init_[i].handler_pointer!= NSD)
		{
			OBJ_MODEL_CLASS.OBJ_HANDLERS[_model_init_[i].id] = (void(*)(void*))_model_init_[i].handler_pointer;
		}
	}
}
/*
	create object function
*/

void obj_soft_create( int obj_id, int obj_class )
{
	if(obj_id > num_of_all_obj)
	{
		return;	
	}
	else
	{
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_ID.object_class = obj_class;
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_TYPE.soft = obj_soft;
	}
}

#ifdef USE_HWOBJ
void  obj_hardware_create( int obj_id, int obj_class,int hwobj )
{
	if(obj_id > num_of_all_obj)
	{
		return;	
	}
	else
	{
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_ID.object_class = obj_class;
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_TYPE.hardware = obj_hard;
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_BIND.HW_adress = hwobj; 		/*snap obj bind field with array of hwobj*/
		OBJ_MODEL_CLASS.HW_OBJ[hwobj] = (OBJ_MODEL_CLASS.objDefault + obj_id);
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ->OBJ_STATUS.hardware.en = TRUE;			/*enable hardware port*/
	}
}
#endif

#ifdef USE_TIMERS
void  obj_timer_create( int obj_id, int obj_class,uint16_t delay,void (*handler_pointer)(OBJ_STRUCT_TypeDef*) )
{
	static int num_of_timer = 1;
	
	if(obj_id > num_of_all_obj)
	{
		return;	
	}
	else
	{
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_ID.object_class = obj_class;
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_TYPE.timer = obj_timer;
		OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_BIND.TimerID = num_of_timer;	/*snap obj bind field with array of timers*/ 
		OBJ_MODEL_CLASS.obj_timers[num_of_timer] = xTimerCreate("",delay,FALSE,
		(void*)&OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_BIND.TimerID,(void(*)(void*))handler_pointer);
		num_of_timer++;	
	}
}
#endif

void obj_model_thread( void )
{
	int i = 0;
	for(i = 1; i < ( num_of_all_obj + 1); i++)
	{
		obj_sync(OBJ_MODEL_CLASS.objDefault + i);
	}
}

void obj_sync( OBJ_STRUCT_TypeDef *instance )
{	
	if(instance->OBJ_SYNC.status.byte & obj_event_mask)             /*if event trigger enable call event function */
	{	
		instance->OBJ_SYNC.status.byte &= ~obj_event_mask;          /*trigger reset*/
		instance->OBJ_STATUS.byte = instance->OBJ_SYNC.status.byte; /*sync external and internal status*/
		obj_event_fnct(instance->OBJ_ID.object_id);
	}
}

/* object event function, result depends from obj type */
void obj_event_fnct( int obj_id )
{
	if(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_ID.object_class != 0)
	{
		#ifdef USE_HWOBJ
		if(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_TYPE.hardware == obj_hard)
		{
			HWOBJ_Event(obj_id);
		}
		#endif
		#ifdef USE_TIMERS
		if((OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_BIND.TimerID != 0)
		&&(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_TYPE.timer == obj_timer))
		{
			xTimerStart(OBJ_MODEL_CLASS.obj_timers[OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_BIND.TimerID],0);
			return;
		}
		#endif
		/* call obj handler */
		OBJ_MODEL_CLASS.OBJ_HANDLERS[obj_id](OBJ_MODEL_CLASS.objDefault + obj_id);
	}
}

void sp_mes_receive( USART_FRAME_TypeDef *mes )
{
	int i = 0;
	uint8_t permission = 0;
	uint16_t _CRC = 0;
	OBJ_STRUCT_TypeDef *obj = OBJ_MODEL_CLASS.objDefault + mes->d_struct.OBJ_ID.object_id;
	
	for(i = LEN_START; i < LEN_USART_MSG_OBJ - (LEN_CRC + LEN_STOP); i++)
	{
		_CRC += mes->byte[i];
	}
	if(_CRC != mes->d_struct.crc)
	{
		return;		/*error crc do not match*/

	}
	permission  = !(mes->d_struct.OBJ_ID.object_class ^ IND_obj_CWS);
	permission |= !(mes->d_struct.OBJ_ID.object_class ^ IND_obj_CAS);
	permission &= !(mes->d_struct.OBJ_ID.object_class ^ obj->OBJ_ID.object_class); /*types are the same*/

	//	/*extended data field*/
//	if(mes->d_struct.object.class_of_obj == IND_obj_COM)
//	{	
//		obj->dWordL = mes->d_struct.object.dWordL;
//		obj->dWordH = mes->d_struct.object.dWordH;	
//		OBJ_Event(mes->d_struct.object.idof_obj);
//		return;		
//	}
	
	if(permission)
	{
		/*sync status and extra field on event trigger*/
		if(mes->d_struct.OBJ_SYNC.status.byte & obj_event_mask)
		{
			obj->OBJ_SYNC = mes->d_struct.OBJ_SYNC;	
		}
		/*add value update trigger*/
		obj->OBJ_VALUE = mes->d_struct.OBJ_VALUE;
	}
}

#ifdef USE_SERIAL_PORT
void sp_all_obj_sync( void )
{
	USART_FRAME_TypeDef object_frame;
	USART_FRAME_TypeDef *usart_memory_pointer;
	int obj_counter = 0;
	uint16_t _CRC_ = 0;
	
	/*pointer to memory space of USART frame*/
	usart_memory_pointer =(USART_FRAME_TypeDef*)OBJ_MODEL_CLASS.USART_DATA;		
	
	/*fill ID and NETWORK*/
	object_frame.d_struct.id_netw = ID_NETWORK;
	object_frame.d_struct.id_modul = ID_DEVICE;
	object_frame.d_struct.start_seq[0] = (uint8_t)START_BYTE_0;
	object_frame.d_struct.start_seq[1] = (uint8_t)START_BYTE_1;
	object_frame.d_struct.stop_seq = (uint8_t)STOP_BYTE;

	/*fill data field*/
	for(int counter = 1; counter < num_of_all_obj; counter ++ )
	{
		_CRC_ = 0;	
		if(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[counter].OBJ_ID.object_class == 0)
		{
			if(obj_counter > num_of_obj)
			{
				break;
			}
			else
			{
				continue;
			}
		}
		/*
		send as obj info field id ,status and value
		*/
		object_frame.d_struct.OBJ_ID = OBJ_MODEL_CLASS.OBJ_AREA.OBJ[counter].OBJ_ID;
		object_frame.d_struct.OBJ_SYNC.status.byte = OBJ_MODEL_CLASS.OBJ_AREA.OBJ[counter].OBJ_STATUS.byte;
		object_frame.d_struct.OBJ_VALUE = OBJ_MODEL_CLASS.OBJ_AREA.OBJ[counter].OBJ_VALUE;
		
		/*fill CRC field*/
		for(int i = 0 + LEN_START; i < LEN_USART_MSG_OBJ - (LEN_CRC + LEN_STOP); i++)
		{
			_CRC_ += object_frame.byte[i];
		}
		object_frame.d_struct.crc = _CRC_;
		/*copy object frame to memory*/
		memcpy(usart_memory_pointer,&object_frame,sizeof(USART_FRAME_TypeDef));
		usart_memory_pointer++;
		obj_counter++;
	}
	#if RTOS_USAGE == TRUE
	/*mutex return in dma transfer complete interrupt*/
	xSemaphoreTake(xMutex_USART_BUSY,portMAX_DELAY);
	#endif
	send_usart_message(OBJ_MODEL_CLASS.USART_DATA,sizeof(USART_FRAME_TypeDef)*obj_counter);	// transfer data to usart
}
#endif
/*----------------------------------------------------------------------*/

/************************task creation functions*************************/

/*task creation function for object model*/
void OBJ_task_init(OBJ_MODEL_PRIORITY *task_priority,int tick_update_rate)
{
	obj_model_init();
	obj_model_setup();
#if RTOS_USAGE == TRUE	
	#if USART_COM_ENABLE == TRUE
		xMutex_USART_BUSY = xSemaphoreCreateMutex();
		usart_receive_buffer = xQueueCreate(MES_BUF_SIZE,sizeof(USART_FRAME_TypeDef));	
	#endif
	xTaskCreate(_task__OBJ_model_thread,"main loop",task_priority->stack_user, NULL,task_priority->user_priority, NULL );
	xTaskCreate(_task__OBJ_data_rx,"rx handler",task_priority->stack_tx_rx, NULL,task_priority->rx_priority, NULL );
#endif	
}

/*software core of object model (1 ms tick)*/
void _task__OBJ_model_thread (void *pvParameters){
	static volatile int tick = 0;
	static const int overload = 3600000UL;
	
	for(;;){
		
		obj_model_task(tick);
		if(tick <= overload ){
			tick++;
		}
		else{
			tick = 0;
		}
		obj_model_thread();
		vTaskDelay(tick_1ms);		
	}
}

/*receive thread of the object model*/
void _task__OBJ_data_rx (void *pvParameters){

#if USART_COM_ENABLE == TRUE
	USART_FRAME_TypeDef buf_usart;
#endif
	
	for(;;)
	{
#if USART_COM_ENABLE == TRUE
		xQueueReceive(usart_receive_buffer,&buf_usart,portMAX_DELAY);
		sp_mes_receive(&buf_usart);
#endif
	}	
}
/************************weak function*************************/
/*usart message*/
void __attribute__((weak)) send_usart_message(uint8_t *message,uint32_t buf_size)
{
	
}
/*obj model setup, config usart update rate, config object initial state */
void __attribute__((weak)) obj_model_setup(void)
{

}
/*obj model loop*/
void __attribute__((weak)) obj_model_task(int tick)
{
}
/*empty handler*/
void __attribute__((weak)) Dummy_Handler(OBJ_STRUCT_TypeDef *obj)
{	
}

void __attribute__((weak)) HWOBJ_Event(int obj_id) 
{
}


