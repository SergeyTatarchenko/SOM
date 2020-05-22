/************************************************************************
* File Name          : OBJ_MODEL.h
* Author             : Tatarchenko S.
* Version            : v 1.6.2
* Description        : Header for OBJ_MODEL.c
*************************************************************************/
#ifndef OBJ_MODEL_H_
#define	OBJ_MODEL_H_
/*----------------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
/*----------------------------------------------------------------------*/
#define MAX_OBJ	0xFF
/*----------------------------------------------------------------------*/

#include "model_config.h"
#include "memory_config.h"


#ifndef num_of_all_obj
    #define num_of_all_obj MAX_OBJ
    #ifndef NUM_OF_HWOBJ
        #define  NUM_OF_HWOBJ MAX_OBJ
    #endif
    #ifndef	NUM_OF_TIMER
        #define  NUM_OF_TIMER MAX_OBJ
    #endif
#endif

/*
*enums to describe types and priorities of objects 
*/

typedef enum{
    IND_obj_SWC = 1,    /*status without control, low priority          */
    IND_obj_CWS = 2,    /*control without control, medium priority      */
    IND_obj_CAS = 3,    /*control with status, high priority            */
}OBJECT_CLASS;

typedef enum{
    obj_soft =  0,  /*software object, default object                   */
    obj_hard  = 1,  /*hardware object, use special hanlder              */
    obj_timer = 2,  /*software timer (with RTOS_USAGE only)             */
}OBJECT_TYPE;

/*------------------------------------------------------------------------
**********************OBJECT STRUCTURE DESCRIPTION************************
------------------------------------------------------------------------*/
#pragma pack(push,1)
typedef struct{
    uint8_t object_id;      /*object id in memory space                 */
    uint8_t object_class;   /*object class and priority                 */
}OBJ_ID_TypeDef;

typedef union{
    uint8_t byte;       /*status field, control and view object state   */

    struct{
        unsigned bit_0:1;
        unsigned bit_1:1;
        unsigned bit_2:1;
        unsigned bit_3:1;
        unsigned bit_4:1;
        unsigned bit_5:1;
        unsigned bit_6:1;
        unsigned bit_7:1;		
    }def;

    struct{
        unsigned state : 1; /*state bit, display current object status  */
        unsigned event : 1; /*event bit setting calls object handler    */
        unsigned ext   : 1; /*extended value (0 - default, 1 - extended)*/
        unsigned txt   : 1; /*enable txt block update (if exist)        */
        unsigned rez4  : 1; /*reserve bit (add new features)            */
        unsigned rez5  : 1; /*reserve bit (add new features)            */
        unsigned rez6  : 1; /*reserve bit (add new features)            */
        unsigned rez7  : 1; /*reserve bit (add new features)            */
    }soft;

    struct {
        unsigned state : 1; /*                   -//-                   */
        unsigned event : 1; /*                   -//-                   */
        unsigned rez2  : 1; /*reserve bit (add new features)            */
        unsigned rez3  : 1; /*reserve bit (add new features)            */
        unsigned rez4  : 1; /*reserve bit (add new features)            */
        unsigned view  : 1; /*indicate current hardware state           */
        unsigned value : 1; /*enable value update from hw               */
        unsigned en    : 1; /*state <-> view sync enable                */
    }hardware;

}OBJ_STATUS_TypeDef;

typedef struct {
    union{
        uint8_t byte;                   /* sync with object status byte */
        struct{
            unsigned bit_0:1;
            unsigned bit_1:1;
            unsigned bit_2:1;
            unsigned bit_3:1;
            unsigned bit_4:1;
            unsigned bit_5:1;
            unsigned bit_6:1;
            unsigned bit_7:1;
        }bit;
    }status;
    uint8_t extra[2];                     /* sync extra byte for future */
}SYNC_TypeDef;

typedef union{
    uint8_t soft;           /*use status field soft                     */
    uint8_t hardware;       /*use status field hardware                 */
    uint8_t timer;          /*use status field soft                     */
}OBJ_TYPE_TypeDef;

typedef union{
    uint8_t HW_adress;/*binding to array of hardware objects (255 max)  */
    uint8_t TimerID;  /*binding to array of software timers  (255 max)  */
}OBJ_BIND_TypeDef;

typedef union{ 
    uint32_t value;                 /*value field, size - 32 bit        */
    struct{
        uint16_t reserve;           /*reserve value (add new features)  */
        uint16_t default_value;     /*default value, size - 16 bit      */
    }def;
    struct{
        uint32_t extended_value;    /*extended value, size - 32 bit     */
    }extended_value;
    struct{
        uint8_t num_of_pages;       /*text block, max page size - 255   */
        uint8_t page_number;        /*text block, current page number   */
        uint16_t page_content;      /* info page , size - 16 bit        */
    }info_block;
}OBJ_VALUE_TypeDef;

typedef struct{
    OBJ_ID_TypeDef OBJ_ID;              /*           16 bit (2 byte)    */
    OBJ_STATUS_TypeDef OBJ_STATUS;      /*            8 bit (1 byte)    */
    SYNC_TypeDef OBJ_SYNC;              /*           24 bit (3 byte)    */
    OBJ_TYPE_TypeDef OBJ_TYPE;          /*            8 bit (1 byte)    */
    OBJ_BIND_TypeDef OBJ_BIND;          /*            8 bit (1 byte)    */
    OBJ_VALUE_TypeDef OBJ_VALUE;        /*           32 bit (4 byte)    */
}OBJ_STRUCT_TypeDef;


#define LEN_START   2
#define LEN_NETW    1
#define LEN_ID      1
#define	LEN_INDEX   2
#define LEN_DATA    8
#define	LEN_CRC     2
#define	LEN_STOP    1
#define LEN_HEAD_SIZE           (LEN_START +     LEN_NETW + LEN_ID   )
#define LEN_OBJ                 (LEN_INDEX +     LEN_DATA + LEN_CRC  )
#define	LEN_USART_MSG_OBJ       (LEN_HEAD_SIZE + LEN_OBJ  + LEN_STOP )

typedef union{
    uint8_t byte[LEN_USART_MSG_OBJ];    /* usart frame array (17 byte)  */
    struct{
        uint8_t start_seq[LEN_START];           /*start 16 bit (2 byte) */
        uint8_t id_netw;        /*current object network 8 bit (1 bite) */
        uint8_t id_modul; /*current module id in network 8 bit (1 bite) */
        OBJ_ID_TypeDef OBJ_ID;                       /* 16 bit (2 byte) */
        SYNC_TypeDef OBJ_SYNC;                       /* 24 bit (3 byte) */
        OBJ_TYPE_TypeDef OBJ_TYPE;                    /* 8 bit (1 byte) */
        OBJ_VALUE_TypeDef OBJ_VALUE;                 /* 32 bit (4 byte) */
        uint16_t crc;       /*check sum for usart frame 16 bit (2 byte) */
        uint8_t stop_seq;                         /*stop 8 bit (1 byte) */
    }d_struct;
}USART_FRAME_TypeDef;

#pragma pack(pop)



typedef struct {
    OBJ_STRUCT_TypeDef *objDefault;		/*pointer to the beginning 
                                        of the object model*/
    union{
        uint8_t OBJ_MEMORY_AREA[sizeof(OBJ_STRUCT_TypeDef)*num_of_all_obj];
        OBJ_STRUCT_TypeDef OBJ[num_of_all_obj];
    }OBJ_AREA;
    void ((*OBJ_HANDLERS[num_of_all_obj+1]))(void*);/*array of pointers to 
                                                     object handler functions*/
#ifdef USE_HWOBJ
    OBJ_STRUCT_TypeDef *HW_OBJ[NUM_OF_HWOBJ];
#endif

#ifdef USE_TIMERS 
    TimerHandle_t obj_timers[NUM_OF_TIMER];
#endif

#ifdef USE_SERIAL_PORT	
    uint8_t USART_DATA[sizeof(USART_FRAME_TypeDef)*num_of_all_obj];
#endif

    unsigned char *text_blocks[num_of_all_obj];
}OBJ_MODEL_CLASS_TypeDef;

typedef struct {
    uint8_t id;                                        /* id (0xFF max) */
    OBJECT_CLASS obj_class;                         /* snap to priority */
    OBJECT_TYPE obj_type;                            /* obj type config */
    uint8_t HW_adress;                         /* snap to hardware enum */
    uint16_t delay;                    /* obj start delay (timers only) */
    void (*handler_pointer)(OBJ_STRUCT_TypeDef*); /* pointer to handler */
}OBJ_INIT_TypeDef;

/*common define*/

#define OBJ_STRUCT                      OBJ_STRUCT_TypeDef
#define idof_obj                        OBJ_ID.object_id      /*id macro*/
#define class_of_obj                    OBJ_ID.object_class
#define obj_status                      OBJ_STATUS.byte   /*status macro*/
#define obj_state                       OBJ_STATUS.soft.state
#define obj_event                       OBJ_STATUS.soft.event
#define extended_value                  OBJ_STATUS.soft.ext
#define obj_view                        OBJ_STATUS.hardware.viev
#define obj_upd_value                   OBJ_STATUS.hardware.value
#define obj_hw_sync                     OBJ_STATUS.hardware.en
#define obj_value                       OBJ_VALUE.value    /*value macro*/
#define obj_def_value                   OBJ_VALUE.def.default_value
#define obj_ext_value                   OBJ_VALUE.extended_value.extended_value
#define obj_text_page_content           OBJ_VALUE.info_block.page_content
#define obj_text_page_number            OBJ_VALUE.info_block.page_number
#define obj_text_num_of_pages           OBJ_VALUE.info_block.num_of_pages
#define obj_type_soft                   OBJ_BIND.soft      /*bind macro */
#define obj_type_hardware               OBJ_BIND.hardware
#define obj_type_timer                  OBJ_BIND.timer
/*----------------------------------------------------------------------*/
#define this_obj(_obj)                  (OBJ_MODEL_CLASS.objDefault + _obj)
#define OBJ(obj)                        OBJ_MODEL_CLASS.OBJ_AREA.OBJ[obj]
#define obj_state_on(obj_id)            this_obj(obj_id)->OBJ_SYNC.status.byte |= obj_status_mask
#define obj_state_off(obj_id)           this_obj(obj_id)->OBJ_SYNC.status.byte &= ~obj_status_mask
#define state_of_obj(obj)               this_obj(obj)->obj_state
#define value_of_obj(obj)               this_obj(obj)->obj_def_value
/*----------------------------------------------------------------------*/
#define SET_OBJ_EVENT_TRIGGER(obj_id)   OBJ(obj_id).OBJ_SYNC.status.byte |= obj_event_mask
#define FORCED_HANDLER_CALL(id)         OBJ_MODEL_CLASS.OBJ_HANDLERS[id](OBJ_MODEL_CLASS.objDefault + id)
#define SET_OBJ_TXT_UPDATE_EN(id)       OBJ(id).OBJ_STATUS.soft.txt = 1
#define SET_OBJ_TXT_UPDATE_DIS(id)      OBJ(id).OBJ_STATUS.soft.txt = 0
#define OBJ_Event(id)                   SET_OBJ_EVENT_TRIGGER(id)

/*-----------------common functions prototypes--------------------------*/
void obj_model_init( void );
void obj_bind( OBJ_INIT_TypeDef* _model_init_,int _model_size_ );
void obj_soft_create( int obj_id, int obj_class );
void obj_hardware_create( int obj_id, int obj_class,int hwobj );
void obj_timer_create( int obj_id, int obj_class,uint16_t delay,void (*handler_pointer)(OBJ_STRUCT_TypeDef*) );
void obj_sync( OBJ_STRUCT_TypeDef *instance );
void obj_model_thread( void );
void obj_model_loop( int tick );
void obj_event_fnct( int obj_id );
uint8_t obj_bind_txt_block(unsigned char *text_block,int text_block_size,int obj_id);
/*--------------serial port functions prototypes------------------------*/
void sp_all_obj_sync( void );
void sp_mes_receive( USART_FRAME_TypeDef *mes );
uint16_t sp_calc_crc( USART_FRAME_TypeDef *mes );
/*----------------------------------------------------------------------*/
#define obj_event_mask                  0x02
#define obj_status_mask                 0x01
/*----------------------------------------------------------------------*/
#define START_BYTE_0    0xAA
#define START_BYTE_1    0x55
#define STOP_BYTE       0xFF
/*----------------------------------------------------------------------*/
#define NSD 0           // define for not used options in OBJ_INIT struct
/*----------------------------------------------------------------------*/
#define tick_1ms    1
#define tick_5ms    5
#define tick_10ms   10
#define tick_25ms   25
#define tick_50ms   50
#define tick_100ms  100
#define tick_250ms  250
#define tick_500ms  500
#define tick_1s     1000
/*----------------------------------------------------------------------*/
typedef struct {
 uint32_t StdId;                                          /*standart ID */
 uint32_t ExtId;                                          /*extended ID */
 uint8_t IDE;                                  /*type of can message ID */
 uint8_t DLC;                                  /*data lenght of message */
 uint8_t Data[8];                                          /*data array */
} CAN_MSG_TypeDef;
/*----------------------------------------------------------------------*/

/*---------------------------------------------------------------------
*************************COMMON VARIABLES******************************
----------------------------------------------------------------------*/
extern OBJ_MODEL_CLASS_TypeDef OBJ_MODEL_CLASS;
extern uint32_t num_of_obj;

#ifdef USE_SERIAL_PORT
    #ifdef USE_RTOS
        extern xSemaphoreHandle xMutex_USART_BUSY;
        extern xQueueHandle usart_receive_buffer;
    #endif
#endif

#ifdef USE_CAN_BUS
    #ifdef USE_RTOS
        extern xQueueHandle can_receive_buffer;
    #endif
#endif

/*---------------------------------------------------------------------
****************OBJ MODEL FUNCTION PROTOTYPES**************************
----------------------------------------------------------------------*/
void _task__OBJ_model_thread (void *pvParameters);
void _task__OBJ_data_rx (void *pvParameters);
void _task__can_data_rx (void *pvParameters);
/*---------------------------------------------------------------------
****************SPECIFIC COMMUNICATION FUNCTIONS***********************
----------------------------------------------------------------------*/
void send_usart_message(uint8_t *message,uint32_t buf_size);
/*---------------------------------------------------------------------
************************WEAK FUNCTIONS*********************************
----------------------------------------------------------------------*/
void HWOBJ_Event(int obj_id);               /*hardware event handler*/
void Dummy_Handler(OBJ_STRUCT_TypeDef *obj);/*empty handler (can be changed)*/
void obj_model_setup(void);                 /*obj model setup, call before thread init*/
void obj_model_task(int tick);              /*obj model loop */
/*----------------------------------------------------------------------*/
#endif
/****************************end of file ********************************/
