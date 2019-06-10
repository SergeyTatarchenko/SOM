/*************************************************
* File Name          : OBJ_MODEL.c
* Author             : Tatarchenko S.
* Version            : v 1.2
* Description        : simple obj model BKM
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
void ((*obj_handlers[num_of_all_obj+1]))(OBJ_STRUCT*);
/*number of objects created*/
uint32_t num_of_obj;
/*array of pointers to hardware objects*/
OBJ_STRUCT *HW_OBJ[NUM_OF_HWOBJ];
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
uint8_t usart_irq_counter;

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
		obj_handlers[counter]= Dummy_Handler;
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
		if(this_obj(i)->id[1]!=0){
			num_of_obj++;
		}
	}
}

/*object creating and snap*/
void obj_snap(obj_init_struct* _model_init_,int _model_size_){
	
	for(int i = 0;i<(_model_size_/sizeof(obj_init_struct));i++){
		if(_model_init_[i].obj_type == obj_hard){
			HWObj_Create(_model_init_[i].id,_model_init_->obj_class,_model_init_[i].HW_adress);
		}else if(_model_init_[i].obj_type == obj_soft){
			Obj_Create(_model_init_[i].id,_model_init_[i].obj_class);
		}
		if(_model_init_[i].handler_pointer!= NULL){
			obj_handlers[_model_init_[i].id] = _model_init_[i].handler_pointer;
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

/*create hardware object, return pointer to obj */
OBJ_STRUCT* HWObj_Create(int obj_id, int obj_type,int hwobj ){
	
	OBJ_STRUCT* obj = Obj_Create(obj_id,obj_type);
	HW_OBJ[hwobj]= this_obj(obj_id); 
	obj->hardware_adress = hwobj;
	obj->obj_hardware = TRUE;
	return obj; 
}

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

void HWOBJ_Event(int obj_id){

#if TARGET != 0	
	OBJ_STRUCT* obj;
	obj = objDefault + obj_id;

	/*output event*/
	if((obj->hardware_adress >= out_0)&&((obj->hardware_adress <= out_7))){
			Set_IO_State((int)(obj->hardware_adress - out_offset),(int)obj->obj_state);
	}
#endif	   
}

/* object event, call object handler and call update function, if event = 1 */
void OBJ_Event(int obj_id){
	
	if(this_obj(obj_id)->typeof_obj != 0){
		if(this_obj(obj_id)->obj_hardware == TRUE){
			HWOBJ_Event(obj_id);		
		}
		obj_handlers[obj_id](this_obj(obj_id));
		/*feedback*/
		
		if(this_obj(obj_id)->obj_event == 1){
			this_obj(obj_id)->obj_event = 0;
			#if ((USART_DATA_FAST == FALSE)&&(MODE == USART_MODE))
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
		if(this_obj(counter)->id[1]!=0){
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
		/*fill object field*/
		pointer = (uint8_t*)&object_frame;
		pointer += (sizeof(object_frame.d_struct.id_netw)+sizeof(object_frame.d_struct.id_modul));
	/*fill array of USART obj*/
	for(int counter = 1; counter < num_of_all_obj; counter ++ ){
		_CRC_ = 0;
		
		if((this_obj(counter)->id[1]==0)/*||(this_obj(counter)->obj_visible == FALSE)*/){
			if(obj_counter > num_of_obj){
				break;
			}else{
				continue;
			}
		}
		memcpy(pointer,this_obj(counter),sizeof(OBJ_STRUCT));
		/*fill CRC field*/
		for(int i = 0; i < LEN_USART_MSG_OBJ - LEN_CRC; i++){
			_CRC_ += object_frame.byte[i];
		}
		object_frame.d_struct.crc = _CRC_;
		/*copy object frame to memory*/
		memcpy(usart_memory_pointer,&object_frame,sizeof(USART_FRAME));
		usart_memory_pointer++;
		obj_counter++;
	}
	/*mutex return in dma transfer complete interrupt*/
	xSemaphoreTake(xMutex_USART_BUSY,portMAX_DELAY);
	send_usart_message(USART_DATA,sizeof(USART_FRAME)*obj_counter);	// transfer data to usart
}
#endif

/* Receive Data Obj */
void Rx_OBJ_Data(USART_FRAME *mes){
	
	int id;
	int i;
	uint8_t type;
	uint16_t _CRC = 0;
	OBJ_STRUCT *obj;
	uint8_t *pointer;
	
	/*id of rec object*/
	id = mes->d_struct.object.id[0];
	/*type of rec object*/
	type = mes->d_struct.object.id[1];
	
	obj = objDefault + id;
	
	for(i = 0; i < (LEN_USART_MSG_OBJ - LEN_CRC); i++)
	{
		_CRC += mes->byte[i];
	}
	
	if(_CRC != mes->d_struct.crc){
		/*error crc do not match*/
		return;
	}
	/*board control object*/
	if(id ==  (IND_obj_NULL + 1)){
		this_obj(obj_STATUS)->status_field = mes->d_struct.object.status_field;
		OBJ_Event(obj_STATUS);
		return;
	}
	if(board_state.bit.power == 1){
		/*receive data (obj type 4) */
		if(type == IND_obj_COM){
			/*take new object image*/
			pointer = (uint8_t*)mes;
			pointer += (sizeof(mes->d_struct.id_netw)+sizeof(mes->d_struct.id_modul));
			memcpy(obj,pointer,sizeof(OBJ_STRUCT));
			OBJ_Event(id);
			return;		
		}
			/*object event*/
		if(mes->d_struct.object.obj_field.default_field.control_byte.byte & event_mask){
			/*if it is a control object*/	
			if((type == obj->id[1])&&(type&IND_obj_CAS)){
			/*take new object image*/
			pointer = (uint8_t*)mes;
			obj->status_field = mes->d_struct.object.status_field;	
			obj->obj_value = mes->d_struct.object.obj_value;
			/*event bit call object handler*/
			OBJ_Event(id);
			/*call obj handler,change event bit on feedback*/	
			}	
		}
	}
}


__weak void send_usart_message(uint8_t *message,uint32_t buf_size){
	
}
void Dummy_Handler(OBJ_STRUCT *obj){
	
}

