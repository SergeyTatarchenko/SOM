/*************************************************
* File Name          : OBJ_MODEL.c
* Author             : Tatarchenko S.
* Version            : v 1.5.1
* Description        : Simple Obj Model 
*************************************************/
#include "OBJ_MODEL.h"
/*----------- global variables-------------------*/

/*global memory array of objects*/
static uint8_t obj_mem_area[sizeof(OBJ_STRUCT)*num_of_all_obj];
/*pointer to the beginning of the model*/
OBJ_STRUCT *objDefault;
/*structure with current board mode*/
BOARD_STATE	board_state;
/*array of pointers to object handler functions*/
void ((*obj_handlers[num_of_all_obj+1]))(void*);
/*number of objects created*/
uint32_t num_of_obj;
#if HARDWARE_OBJECT == TRUE
/*array of pointers to hardware objects*/
OBJ_STRUCT *HW_OBJ[NUM_OF_HWOBJ];
#endif
/*tasks init struct*/
OBJ_MODEL_PRIORITY task_priority;

#if OBJECT_TIMER == TRUE
	TimerHandle_t obj_timers[NUM_OF_TIMER];
#endif

#ifdef USART_COM_ENABLE
	/*pointer to an array of frames in the message for USART*/
	uint8_t USART_DATA[sizeof(USART_FRAME)*num_of_all_obj];
	/* data array for usart obj transfer */
	uint8_t	usart_data_transmit_array[USART1_DEFAULT_BUF_SIZE];
	uint8_t	usart_data_stream[USART_STREAM_SIZE];
	/* data array for usart obj receive */
	uint8_t usart_data_receive_array[USART1_DEFAULT_BUF_SIZE];
	/*mutex  to perform currect usart transmit */
	xSemaphoreHandle xMutex_USART_BUSY;
	/*queue of messages from usart module*/
	xQueueHandle usart_receive_buffer;
	/*usart data byte counter */
	uint8_t usart_irq_counter = 0;
#endif

/*-----------------------------------------------*/

/*init obj model*/
void OBJ_Init(){
	
	OBJ_STRUCT *obj;
    obj_init_struct _model_init_[] ={_obj_cofig_};
	/* object memory allocation*/
	memset(obj_mem_area,0,sizeof(OBJ_STRUCT)*num_of_all_obj);
	objDefault =(OBJ_STRUCT*)obj_mem_area;
	
	for(int counter = 0;counter <= num_of_all_obj;counter++){
		obj = objDefault + counter;
		obj->id[0] = counter;	
	}
	
	for(int counter = 0;counter <= num_of_all_obj;counter++){
		obj_handlers[counter]= (void(*)(void*))Dummy_Handler;
	}
#if HARDWARE_OBJECT == TRUE
		for(int counter = 0;counter <= NUM_OF_HWOBJ;counter++){
		HW_OBJ[counter] = objDefault;
	}
#endif
#if USART_DATA_FAST == TRUE
		memset(USART_DATA,0,sizeof(USART_FRAME)*num_of_all_obj);
#endif	
	/* object create and handler mapping*/
	obj_snap(_model_init_,sizeof(_model_init_));
	/*get current number of objects in memory area */
	num_of_obj = 0;
	for(int i = 0;i<=num_of_all_obj;i++){
		if(this_obj(i)->typeof_obj != 0){
			num_of_obj++;
		}
	}
}

/*object creating and snap*/
void obj_snap(obj_init_struct* _model_init_,int _model_size_){
	for(int i = 0;i<(_model_size_/sizeof(obj_init_struct));i++){
		#if HARDWARE_OBJECT == TRUE
		/*hardware obj create*/
		if(_model_init_[i].obj_type == obj_hard){
			HWObj_Create(_model_init_[i].id,_model_init_->obj_class,_model_init_[i].HW_adress);
		}
		#endif
		/*soft obj create*/
		if(_model_init_[i].obj_type == obj_soft){
			Obj_Create(_model_init_[i].id,_model_init_[i].obj_class);
		}
		#if OBJECT_TIMER == TRUE 
		/*timers creation*/
		if(_model_init_[i].obj_type == obj_timer){
			Timer_Create(_model_init_[i].id,_model_init_[i].obj_type,_model_init_[i].delay,_model_init_[i].handler_pointer);
		}
		#endif
		/*handlers swap*/
		if(_model_init_[i].handler_pointer!= NULL){
			obj_handlers[_model_init_[i].id] = (void(*)(void*))_model_init_[i].handler_pointer;
		}
	}
}

/*create object, return pointer to obj */
OBJ_STRUCT* Obj_Create(int obj_id, int obj_type ){
	
	OBJ_STRUCT* obj;
	obj = objDefault + obj_id;
	obj->id[1] = obj_type;
	obj->obj_visible = TRUE;
	return obj;
}

#if HARDWARE_OBJECT == TRUE
/*create hardware object, return pointer to obj */
OBJ_STRUCT* HWObj_Create(int obj_id, int obj_type,int hwobj ){
	
	OBJ_STRUCT* obj = Obj_Create(obj_id,obj_type);
	HW_OBJ[hwobj]= this_obj(obj_id); 
	obj->hardware_adress = hwobj;
	obj->obj_hardware = TRUE;
	return obj; 
}
#endif

#if OBJECT_TIMER == TRUE
/*create hardware object, return pointer to obj */
OBJ_STRUCT* Timer_Create(int obj_id, int obj_type,uint16_t delay,void (*handler_pointer)(OBJ_STRUCT*)){
	static int num_of_timer = 1;
	OBJ_STRUCT* obj = Obj_Create(obj_id,obj_type);
	obj->timer_adress = num_of_timer;
	obj_timers[num_of_timer] = xTimerCreate("",delay,FALSE,(void*)&obj->timer_adress,(void(*)(void*))handler_pointer);
	num_of_timer++;	
	return obj; 
}
#endif

/*set state with update*/
void OBJ_SetState(int obj_id,int state){
	if(state>1){
		return;
	}
	if(this_obj(obj_id)->obj_state != state){
		this_obj(obj_id)->obj_state = state;
		obj_handlers[obj_id](this_obj(obj_id));
		OBJ_Upd_USART(this_obj(obj_id));
	}
}
void set_all_obj_off(void)
{
	for(int i = 0; i < num_of_obj;i++)
	{
		if((objDefault+i)->typeof_obj != 0)
		{
			obj_state_off(i);
		}
	}
}
/* object event, call object handler and call update function, if event = 1 */
void OBJ_Event(int obj_id){
	
	if(this_obj(obj_id)->typeof_obj != 0){
		if(this_obj(obj_id)->obj_hardware == TRUE){
			HWOBJ_Event(obj_id);		
		}
		#if OBJECT_TIMER == TRUE
		/*timer event*/
		if(this_obj(obj_id)->timer_adress != 0){
			xTimerStart(obj_timers[this_obj(obj_id)->timer_adress],0);
		}
		#endif
		/* default soft object*/
		else{
			obj_handlers[obj_id](this_obj(obj_id));
		}
		
				/*feedback*/
		
		if(this_obj(obj_id)->obj_event == 1){
			this_obj(obj_id)->obj_event = 0;
			#if ((USART_DATA_FAST == FALSE)&&(MODE == USART_COM_ENABLE))
				OBJ_Upd_USART(this_obj(obj_id));
			#endif
		}
	}	
}

/*           update this object             */
void	OBJ_Upd_USART(OBJ_STRUCT *obj){

		uint8_t *pointer;
		USART_FRAME message;
		USART_FRAME *message_pointer;
		uint16_t _CRC_ = 0;
		message_pointer =&message;
	
		/*create default message with obj info*/
		message_pointer->d_struct.id_netw = ID_NETWORK;
		message_pointer->d_struct.id_modul = ID_DEVICE;
	
		pointer = (uint8_t*)message_pointer;
		pointer += (sizeof(message.d_struct.id_netw)+sizeof(message.d_struct.id_modul));
		memcpy(pointer,obj,sizeof(OBJ_STRUCT));
	
		/* Calc check summ */
		for(int i = 0; i < LEN_USART_MSG_OBJ - LEN_CRC; i++){
			_CRC_ += message_pointer->byte[i];
		}
		message_pointer->d_struct.crc = _CRC_;
	
		/*mutex return in dma transfer complete interrupt*/
		xSemaphoreTake(xMutex_USART_BUSY,portMAX_DELAY);
		send_usart_message((uint8_t*)message_pointer,sizeof(USART_FRAME));	// transfer data to usart

	/*message delay for corrent receive */
	vTaskDelay(5);
}

/*             update all obj                */
void Upd_All_OBJ_USART(){
	
	for(int counter = 0; counter < num_of_all_obj; counter ++){
		if(this_obj(counter)->typeof_obj != 0){
			OBJ_Upd_USART(this_obj(counter));
		}
	}
}

/*         fast update all obj with DMA      */
#ifdef	USART_DATA_FAST
void FAST_Upd_All_OBJ_USART(void){
	
	USART_FRAME object_frame;
	USART_FRAME *usart_memory_pointer;
	uint8_t *pointer;
	uint16_t _CRC_ = 0;
	int obj_counter = 0;
	
	/*pointer to memory space of USART frame*/
	usart_memory_pointer =(USART_FRAME*)USART_DATA;		
		/*fill ID and NETWORK*/
		object_frame.d_struct.id_netw = ID_NETWORK;
		object_frame.d_struct.id_modul = ID_DEVICE;
		//
		object_frame.d_struct.start_seq[0] = (uint8_t)START_BYTE_0;
		object_frame.d_struct.start_seq[1] = (uint8_t)START_BYTE_1;
		object_frame.d_struct.stop_seq = (uint8_t)STOP_BYTE;
		//
		/*fill object field*/
		pointer = (uint8_t*)&object_frame;
		pointer += (sizeof(object_frame.d_struct.id_netw)+sizeof(object_frame.d_struct.id_modul)+
					sizeof(object_frame.d_struct.start_seq));
	/*fill array of USART obj*/
	for(int counter = 1; counter < num_of_all_obj; counter ++ ){
		_CRC_ = 0;
		
		if(this_obj(counter)->typeof_obj == 0){
			if(obj_counter > num_of_obj){
				break;
			}else{
				continue;
			}
		}
		memcpy(pointer,this_obj(counter),sizeof(OBJ_STRUCT));
		/*fill CRC field*/
		for(int i = 0 + LEN_START; i < LEN_USART_MSG_OBJ - (LEN_CRC + LEN_STOP); i++){
			_CRC_ += object_frame.byte[i];
		}
		object_frame.d_struct.crc = _CRC_;
		/*copy object frame to memory*/
		memcpy(usart_memory_pointer,&object_frame,sizeof(USART_FRAME));
		usart_memory_pointer++;
		obj_counter++;
	}
	/*mutex return in dma transfer complete interrupt*/
	xSemaphoreTake(xMutex_USART_BUSY,500);
	send_usart_message(USART_DATA,sizeof(USART_FRAME)*obj_counter);	// transfer data to usart
}
#endif



/*create CAN message with extended ID, return message*/
CAN_OBJ_FRAME can_obj_create_message (int obj_id){
	CAN_OBJ_FRAME message;
	
	message.id = (((this_obj(obj_id)->id[0])&can_obj_mask)|
				 (((uint32_t)ID_NETWORK<<8)&can_id_mask)  |
				 (((uint32_t)ID_DEVICE<<14)&can_net_mask));
	message.len = 8;
	memcpy(&message.data,&(this_obj(obj_id)->obj_field),sizeof(message.data));
	return message;
}

/* Receive Data Obj with USART */
void Rx_OBJ_Data(USART_FRAME *mes){
	int i = 0;
	uint16_t _CRC = 0;	
	OBJ_STRUCT *obj = objDefault + mes->d_struct.object.idof_obj;

	for(i = 0 + LEN_START; i < LEN_USART_MSG_OBJ - (LEN_CRC + LEN_STOP); i++)
	{
		_CRC += mes->byte[i];
	}
	
	if(_CRC != mes->d_struct.crc){
		/*error crc do not match*/
		return;
	}
	/*------board control object, software special----------*/
	if(mes->d_struct.object.idof_obj == obj_STATUS){
		this_obj(obj_STATUS)->status_field = mes->d_struct.object.status_field;
		OBJ_Event(obj_STATUS);
		return;
	}
	/*-----------------------------------------------------*/
	if(board_power == 1){
		/*extended data field*/
		if(mes->d_struct.object.typeof_obj == IND_obj_COM){
			
			obj->dWordL = mes->d_struct.object.dWordL;
			obj->dWordH = mes->d_struct.object.dWordH;
			
			OBJ_Event(mes->d_struct.object.idof_obj);
			return;		
		}
		/*object event*/
		if(mes->d_struct.object.obj_event == 1){
			if( (mes->d_struct.object.typeof_obj == obj->typeof_obj) && 
				((obj->typeof_obj == IND_obj_CAS)||(obj->typeof_obj == IND_obj_CWS)) )
			{
			obj->status_field = mes->d_struct.object.status_field;	
			obj->obj_value = mes->d_struct.object.obj_value;
			/*event bit call object handler*/
			OBJ_Event(mes->d_struct.object.idof_obj);	
			}	
		}
	}
}
/************************task creation functions*************************/

/*task creation function for object model*/
void OBJ_task_init(OBJ_MODEL_PRIORITY *task_priority,int tick_update_rate)
{
	OBJ_Init();
	obj_model_setup();
#if RTOS_USAGE == TRUE	
	#if USART_COM_ENABLE == TRUE
		xMutex_USART_BUSY = xSemaphoreCreateMutex();
		usart_receive_buffer = xQueueCreate(MES_BUF_SIZE,sizeof(USART_FRAME));	
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
		vTaskDelay(tick_1ms);		
	}
}

/*receive thread of the object model*/
void _task__OBJ_data_rx (void *pvParameters){

#if USART_COM_ENABLE == TRUE
	USART_FRAME buf_usart;
#endif
	
	for(;;){

#if USART_COM_ENABLE == TRUE
		xQueueReceive(usart_receive_buffer,&buf_usart,portMAX_DELAY);
		Rx_OBJ_Data(&buf_usart);
#endif
	}	
}
/************************weak function*************************/
/*usart message*/
__weak void send_usart_message(uint8_t *message,uint32_t buf_size){
	
}
/*CAN message*/
__weak void send_can_message(CAN_OBJ_FRAME message){

}

/*obj model setup, config usart update rate, config object initial state */
__weak void obj_model_setup(void){


}
/*obj model loop*/
__weak void obj_model_task(int tick){


}
/*empty handler*/
__weak void Dummy_Handler(OBJ_STRUCT *obj){
	
}

__weak void HWOBJ_Event(int obj_id){
	   
}


