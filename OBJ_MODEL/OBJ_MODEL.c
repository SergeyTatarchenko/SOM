/************************************************************************
* File Name          : OBJ_MODEL.c
* Author             : Tatarchenko S.
* Version            : v 1.6.2
* Description        : Simple Obj Model 
*************************************************************************/
#include "OBJ_MODEL.h"
/*---------------------------------------------------------------------
*************************COMMON VARIABLES******************************
----------------------------------------------------------------------*/
/*
number of objects created; the variable must not exceed 
the value of num_of_all_obj !!!
*/
uint32_t num_of_obj;

#ifdef USE_RTOS
    #ifdef USE_SERIAL_PORT
        xSemaphoreHandle xMutex_USART_BUSY;
        xQueueHandle usart_receive_buffer;
    #endif
    #ifdef USE_CAN_BUS
        xQueueHandle can_receive_buffer;
    #endif
#endif

OBJ_MODEL_CLASS_TypeDef OBJ_MODEL_CLASS;

/*
* name : obj_model_init
* description : The function of creating an object model (main initialization function)
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
    obj_bind(_model_init_,sizeof(_model_init_));
    num_of_obj = 0;
    for(int i = 0;i<=num_of_all_obj;i++)
    {
        if(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[i].OBJ_ID.object_class != 0)
        {
            num_of_obj++;
            /* !primary status and sync byte synchronization*/
            OBJ_MODEL_CLASS.OBJ_AREA.OBJ[i].OBJ_SYNC.status.byte =
            OBJ_MODEL_CLASS.OBJ_AREA.OBJ[i].OBJ_STATUS.byte;
        }
    }
}

/*
* name : obj_bind
* description: function of placing objects in the model memory 
* according to the input array of structures with OBJ_INIT_TypeDef type
*/
void obj_bind(OBJ_INIT_TypeDef* _model_init_,int _model_size_)
{
    int i = 0;
    uint8_t obj_quantity = _model_size_/sizeof(OBJ_INIT_TypeDef);
    for(i = 0 ; i < obj_quantity ; i++)
   {
        if(_model_init_[i].obj_type == obj_soft)
        {
            obj_soft_create( _model_init_[i].id,_model_init_[i].obj_class );
        }
        #ifdef USE_HWOBJ
        if(_model_init_[i].obj_type == obj_hard)
        {
            obj_hardware_create(_model_init_[i].id,_model_init_[i].obj_class,_model_init_[i].HW_adress);
        }
        #endif
        #ifdef USE_TIMERS
        if(_model_init_[i].obj_type == obj_timer)
        {
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
* name : obj_soft_create
* description : function creates a soft object (standard type)
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

/*
* name : obj_hardware_create
* description: the function creates an object with a binding to the hardware platform 
* with a call to a weak separate handler
*/
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
        OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_BIND.HW_adress = hwobj;
        OBJ_MODEL_CLASS.HW_OBJ[hwobj] = (OBJ_MODEL_CLASS.objDefault + obj_id);
        OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].obj_upd_value = TRUE;
        OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].obj_hw_sync  = TRUE;
    }
}
#endif

/*
* name : obj_timer_create
* description: function creates an object with a timer type (RTOS only)
*/
#ifdef USE_TIMERS
void obj_timer_create( int obj_id, int obj_class,uint16_t delay,void (*handler_pointer)(OBJ_STRUCT_TypeDef*) )
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
        (void*)&OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_BIND.TimerID,(TimerCallbackFunction_t)handler_pointer);
        num_of_timer++;
    }
}
#endif

/*
* name : obj_model_thread
* description: trigger check cycle of all model objects
*/
void obj_model_thread( void )
{
    int i = 0;
    for(i = 1; i < ( num_of_all_obj + 1); i++)
    {
        obj_sync(OBJ_MODEL_CLASS.objDefault + i);
    }
}

/*
* name : obj_sync
* description: the function checks for an event trigger or object state change
*/
void obj_sync( OBJ_STRUCT_TypeDef *instance )
{
    if((instance->OBJ_SYNC.status.byte & obj_status_mask)
    != (instance->OBJ_STATUS.byte & obj_status_mask) )
		{
        instance->OBJ_STATUS.byte = instance->OBJ_SYNC.status.byte;
        obj_event_fnct(instance->OBJ_ID.object_id);
    }
    else if(instance->OBJ_SYNC.status.byte & obj_event_mask)
    {
        instance->OBJ_SYNC.status.byte &= ~obj_event_mask;
        instance->OBJ_STATUS.byte = instance->OBJ_SYNC.status.byte;
        obj_event_fnct(instance->OBJ_ID.object_id);
    }
}

/*
* name : obj_event_fnct
* description: the function raises an object event depending on its type
*/
void obj_event_fnct( int obj_id )
{
    if(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_ID.object_class != 0)
    {
        #ifdef USE_HWOBJ
        if(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].OBJ_TYPE.hardware == obj_hard)
        {
            if(OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].obj_hw_sync == TRUE)
            {
                HWOBJ_Event(obj_id);
            }
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
        OBJ_MODEL_CLASS.OBJ_HANDLERS[obj_id](OBJ_MODEL_CLASS.objDefault + obj_id);
    }
}

/*
* name : obj_bind_txt_block
* description: The function binds a text block to the object value field 
*/
uint8_t obj_bind_txt_block(unsigned char *text_block,int text_block_size,int obj_id)
{
    uint8_t num_of_pages;
    if((text_block_size > 0xFF)&&(*text_block != NULL))	/*data input restriction*/
    {
        return FALSE;
    }
    else
    {
        if(text_block_size % 2 == 0)
        {
            num_of_pages = text_block_size/2; /*even number of pages*/ 
        }
        else
        {
            num_of_pages = text_block_size/2 + 1; /*odd number of pages*/ 
        }
        OBJ_MODEL_CLASS.text_blocks[obj_id] = text_block; /*bind text block to obj array*/
        OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].obj_text_num_of_pages = num_of_pages; 
        OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].obj_text_page_number = 0;
        OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj_id].obj_text_page_content = *(uint16_t*)text_block;
        return TRUE;
    }
}

/*
* name : obj_txt_block_update
* description: function of updating textual data field 
*/
void obj_txt_block_update(int obj_id)
{
}

/*
* name : sp_mes_receive
* description: The function receives data from the message received via the serial port 
* and sets model triggers
*/
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
        return;
    }
    permission  = !(mes->d_struct.OBJ_ID.object_class ^ IND_obj_CWS);
    permission |= !(mes->d_struct.OBJ_ID.object_class ^ IND_obj_CAS);
    permission &= !(mes->d_struct.OBJ_ID.object_class ^ obj->OBJ_ID.object_class); /*types are the same*/
    if(permission)
    {
        /*sync status and extra field on event trigger*/
        if(mes->d_struct.OBJ_SYNC.status.byte != obj->OBJ_SYNC.status.byte)
        {
            obj->OBJ_SYNC = mes->d_struct.OBJ_SYNC;	
        }
        /*add value update trigger*/
        obj->OBJ_VALUE = mes->d_struct.OBJ_VALUE;
    }
}

/*
* name : sp_all_obj_sync
* description: function of sending information about all objects via the serial port
*/
#ifdef USE_SERIAL_PORT
void sp_all_obj_sync( void )
{
    USART_FRAME_TypeDef object_frame;
    USART_FRAME_TypeDef *usart_memory_pointer;
    int obj_counter = 0;
    uint16_t _CRC_ = 0;
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
        /*send as obj info field id ,status and value*/
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
    #ifdef USE_RTOS
        /*mutex return in dma transfer complete interrupt*/
        xSemaphoreTake(xMutex_USART_BUSY,portMAX_DELAY);
    #endif
    send_usart_message(OBJ_MODEL_CLASS.USART_DATA,sizeof(USART_FRAME_TypeDef)*obj_counter);	// transfer data to seria port
}
#endif

/*
* name : OBJ_task_init
* description: function to create tasks of the object model (RTOS only)
*/
void OBJ_task_init(void *mem_stack,int tick_update_rate)
{
    obj_model_init();
    obj_model_setup();
}

/*
* name : _task__OBJ_model_thread
* description: object model main task (RTOS only)
*/
#ifdef USE_RTOS
void _task__OBJ_model_thread (void *pvParameters)
{
    static volatile int tick = 0;
    static const int overload = 3600000UL;
    for(;;)
    {
        if(tick <= overload )
        {
            tick++;
        }
        else
        {
            tick = 0;
        }
        obj_model_loop(tick);
        vTaskDelay(tick_1ms);
    }
}
#endif

/*
* name : obj_model_loop
* description: object model main loop
*/
void obj_model_loop( int tick )
{
    obj_model_thread();
    obj_model_task(tick);
}

/*
* name : _task__OBJ_data_rx
* description: message processing task of object model (RTOS only)
*/
void _task__OBJ_data_rx (void *pvParameters)
{
    #ifdef USE_SERIAL_PORT
        USART_FRAME_TypeDef buf_usart;
    #endif
    for(;;)
    {
        #ifdef USE_RTOS
            #ifdef USE_SERIAL_PORT
                xQueueReceive(usart_receive_buffer,&buf_usart,portMAX_DELAY);
                sp_mes_receive(&buf_usart);
            #endif
        #endif
    }
}

/*-----------------------Weak functions-------------------------------*/

/*
* name : send_usart_message
* description: function of sending messages via serial port 
* (weak, may be defined elsewhere)
*/
void __attribute__((weak)) send_usart_message(uint8_t *message,uint32_t buf_size)
{
}

/*
* name : obj_model_setup
* description: initial settings function, called after model initialization
* (weak, may be defined elsewhere)
*/
void __attribute__((weak)) obj_model_setup(void)
{
}

/*
* name : obj_model_task
* description: model subtask, any necessary cyclic executable code
* (weak, may be defined elsewhere)
*/
void __attribute__((weak)) obj_model_task(int tick)
{
}

/*
* name : Dummy_Handler
* description: empty handler
* (weak, may be defined elsewhere)
*/
void __attribute__((weak)) Dummy_Handler(OBJ_STRUCT_TypeDef *obj)
{
}

/*
* name : HWOBJ_Event
* description: a separate handler that is called by the hardware object
* (weak, may be defined elsewhere)
*/
void __attribute__((weak)) HWOBJ_Event(int obj_id) 
{
}

/*
* name : sp_calc_crc
* description: crc calculation function
* (weak, may be defined elsewhere)
*/
uint16_t __attribute__((weak))  sp_calc_crc( USART_FRAME_TypeDef *mes ) 
{
    int i = 0;
    uint16_t _CRC = 0;
    for(i = LEN_START; i < LEN_USART_MSG_OBJ - (LEN_CRC + LEN_STOP); i++)
    {
        _CRC += mes->byte[i];
    }
    return _CRC;
}
/*------------------------end of file----------------------------------*/
