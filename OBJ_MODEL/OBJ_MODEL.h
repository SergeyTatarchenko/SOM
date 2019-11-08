/*************************************************************************
* File Name          : OBJ_MODEL.h
* Author             : Tatarchenko S.
* Version            : v 1.6
* Description        : header for OBJ_MODEL.c 
*************************************************************************/
#ifndef OBJ_MODEL_H_
#define	OBJ_MODEL_H_
/*----------------------------------------------------------------------*/
#include "stdint.h"
#include "string.h"
/*----------------------------------------------------------------------*/
#define MAX_OBJ	0xFF
/*----------------------------------------------------------------------*/
/* enums to describe types and priorities of objects                    */
typedef enum{
	IND_obj_SWC = 1,	/*status without control, low priority          */
	IND_obj_CWS = 2,	/*control without control, medium priority      */
	IND_obj_CAS = 3,	/*control with status, high priority            */
	IND_obj_COM = 4		/*object with extended data field, high priority*/
}OBJECT_CLASS;
/*----------------------------------------------------------------------*/
typedef enum{
	obj_soft =  0,	/*software object, default object                   */
	obj_hard  = 1,	/*hardware object, use special hanlder              */
	obj_timer = 2,	/*software timer (with RTOS_USAGE only)             */
	obj_text =  3	/*info object,use extended data field, contains text*/
}OBJECT_TYPE;
/*----------------------------------------------------------------------*/
typedef struct{
	uint8_t system_priority;	/*system layer tasks                    */
	uint8_t tx_priority;		/*transmit data tasks                   */
	uint8_t rx_priority;		/*receive data tasks                    */
	uint8_t user_priority;		/*user tasks                            */
	uint_least16_t stack_tx_rx;	/*stack size for data transmit/receive  */
	uint_least16_t stack_user;	/*stack for user layer tasks and threads*/
	uint8_t tick_update_rate;	/*object model tick update rate         */
}OBJ_MODEL_PRIORITY;
/*----------------------------------------------------------------------*/
#pragma pack(push,1)
typedef union{
	uint8_t status;
	struct{
		unsigned power:1;
		unsigned mode:1;
		unsigned hwobj:1;
		unsigned debug:1;
		unsigned model_on:1;
		unsigned rez5:1;
		unsigned rez6:1;
		unsigned rez7:1;
	}bit;
}BOARD_STATE;
#pragma pack(pop)
/*------------------------------------------------------------------------
**********************OBJECT STRUCTURE DESCRIPTION************************
------------------------------------------------------------------------*/
#pragma pack(push,1)
typedef struct{
	uint8_t object_id;		/*object id in memory space                 */
	uint8_t object_class;	/*object class and priority                 */
}OBJ_ID_TypeDef;
#pragma pack(pop)
/*----------------------------------------------------------------------*/
#pragma pack(push,1)
typedef union{
	uint8_t byte;		/*status field, control and view object state   */
	/*-------------------default field 8 bit----------------------------*/
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
	/*-------------------field for soft object(default)-----------------*/
	struct{
		unsigned state : 1;	/*state bit, display current object status  */
		unsigned event : 1; /*event bit setting calls object handler    */
		unsigned ext   : 1;	/*extended value (0 - default, 1 - extended)*/
		unsigned rez3  : 1; /*reserve bit (add new features)            */
		unsigned rez4  : 1; /*reserve bit (add new features)            */
		unsigned rez5  : 1; /*reserve bit (add new features)            */
		unsigned rez6  : 1; /*reserve bit (add new features)            */
		unsigned rez7  : 1; /*reserve bit (add new features)            */
	}soft;
	/*----------------field for hardware object-------------------------*/
	struct {
		unsigned state : 1; /*                   -//-                   */
		unsigned event : 1; /*                   -//-                   */
		unsigned view  : 1; /*indicate current hardware state           */
		unsigned rez3  : 1; /*reserve bit (add new features)            */
		unsigned rez4  : 1; /*reserve bit (add new features)            */
		unsigned rez5  : 1; /*reserve bit (add new features)            */
		unsigned value : 1; /*enable value update from hw               */
		unsigned en    : 1; /*state <-> view sync enable                */
	}hardware;
}OBJ_STATUS_TypeDef;
#pragma pack(pop)
/*----------------------------------------------------------------------*/
#pragma pack(push,1)
typedef struct {
	union{
		uint8_t byte;				    /* sync with object status byte */
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
	uint8_t extra[2];					  /* sync extra byte for future */
}SYNC_TypeDef;
#pragma pack(pop)
/*----------------------------------------------------------------------*/
#pragma pack(push,1)
typedef union{
	uint8_t soft;			/*use status field soft                     */
	uint8_t hardware;		/*use status field hardware                 */
	uint8_t timer;			/*use status field soft                     */
}OBJ_TYPE_TypeDef;
#pragma pack(pop)
/*----------------------------------------------------------------------*/
#pragma pack(push,1)
typedef union{
	uint8_t HW_adress;/*binding to array of hardware objects (255 max)  */
	uint8_t TimerID;  /*binding to array of software timers  (255 max)  */
}OBJ_BIND_TypeDef;
#pragma pack(pop)
/*----------------------------------------------------------------------*/
#pragma pack(push,1)
typedef union{ 
	uint32_t value;	  				/*value field, size - 32 bit        */
	struct{
		uint16_t reserve;			/*reserve value (add new features)  */
		uint16_t default_value;		/*default value, size - 16 bit      */
	}def;
	struct{
		uint32_t extended_value;	/*extended value, size - 32 bit     */
	}extended_value;
	struct{
		uint8_t num_of_pages;		/*text block, max page size - 255   */
		uint8_t page_number;		/*text block, current page number   */
		uint16_t page_content;	    /* info page , size - 16 bit        */
	}info_block;
}OBJ_VALUE_TypeDef;
#pragma pack(pop)
/*----------------------------------------------------------------------*/
typedef struct{
	/**/
	OBJ_ID_TypeDef OBJ_ID;				/*           16 bit (2 byte)    */
	
	OBJ_STATUS_TypeDef OBJ_STATUS;		/*            8 bit (1 byte)    */
	
	SYNC_TypeDef OBJ_SYNC;				/*           32 bit (3 byte)    */
	
	OBJ_TYPE_TypeDef OBJ_TYPE;			/*            8 bit (1 byte)    */
	
	OBJ_BIND_TypeDef OBJ_BIND;			/*            8 bit (1 byte)    */
	
	OBJ_VALUE_TypeDef OBJ_VALUE;		/*           32 bit (4 byte)    */
}OBJ_STRUCT_TypeDef;
/*----------------------------------------------------------------------*/
#define LEN_START	2
#define LEN_NETW	1
#define LEN_ID		1
#define	LEN_INDEX	2
#define LEN_DATA	8
#define	LEN_CRC		2
#define	LEN_STOP	1
#define LEN_HEAD_SIZE			(LEN_START + LEN_NETW + LEN_ID )
#define LEN_OBJ					(LEN_INDEX + LEN_DATA + LEN_CRC)
#define	LEN_USART_MSG_OBJ		(LEN_HEAD_SIZE + LEN_OBJ + LEN_STOP)
/*----------------------------------------------------------------------*/
#pragma pack(push,1)
typedef union{
	uint8_t byte[LEN_USART_MSG_OBJ];	/* usart frame array (17 byte)  */
	struct{
		uint8_t start_seq[LEN_START];	        /*start 16 bit (2 byte) */
        uint8_t id_netw;		/*current object network 8 bit (1 bite) */
        uint8_t id_modul; /*current module id in network 8 bit (1 bite) */
		OBJ_ID_TypeDef OBJ_ID;						 /* 16 bit (2 byte) */
		SYNC_TypeDef OBJ_SYNC;						 /* 24 bit (4 byte) */
		OBJ_TYPE_TypeDef OBJ_TYPE;					  /* 8 bit (1 byte) */
		OBJ_VALUE_TypeDef OBJ_VALUE;	             /* 32 bit (4 byte) */
        uint16_t crc;		/*check sum for usart frame 16 bit (2 byte) */
		uint8_t stop_seq;                         /*stop 8 bit (1 byte) */
    }d_struct;
}USART_FRAME_TypeDef;
#pragma pack(pop)
/*----------------------------------------------------------------------*/
#include "obj_model_config.h"
/*----------------------------------------------------------------------*/
typedef struct {
	OBJ_STRUCT_TypeDef *objDefault;		/*pointer to the beginning 
										  of the object model			*/
	/*static memory array for objects (size define with num_of_all_obj) */
	union{
		uint8_t OBJ_MEMORY_AREA[sizeof(OBJ_STRUCT_TypeDef)*num_of_all_obj];
		OBJ_STRUCT_TypeDef OBJ[num_of_all_obj];
	}OBJ_AREA;
	/*array of pointers to object handler functions*/
	void ((*OBJ_HANDLERS[num_of_all_obj+1]))(void*);
	#ifdef USE_HWOBJ
	/*array of pointers to hardware objects*/
	OBJ_STRUCT_TypeDef *HW_OBJ[NUM_OF_HWOBJ];
	#endif
	#ifdef USE_TIMERS
	/*array of pointers to RTOS software timers*/
	TimerHandle_t obj_timers[NUM_OF_TIMER];
	#endif
	/*array for transmittion with serial port*/
	#ifdef USE_SERIAL_PORT	
	uint8_t USART_DATA[sizeof(USART_FRAME_TypeDef)*num_of_all_obj];
	#endif
	/*array of pointers to text( max text block size 0xff )*/ 
	unsigned char *text_blocks[num_of_all_obj];
}OBJ_MODEL_CLASS_TypeDef;
/*----------------------------------------------------------------------*/
#pragma pack(push,1)
typedef struct {
	uint8_t id;										   /* id (0xFF max) */
	OBJECT_CLASS obj_class;							/* snap to priority */
	OBJECT_TYPE obj_type;					         /* obj type config */
	uint8_t HW_adress;						   /* snap to hardware enum */
	uint16_t delay;					   /* obj start delay (timers only) */
	void (*handler_pointer)(OBJ_STRUCT_TypeDef*); /* pointer to handler */
}OBJ_INIT_TypeDef;
#pragma pack(pop)
/*----------------------------------------------------------------------*/

/*obj memory usage, use max value for default*/
#ifndef num_of_all_obj
	#define num_of_all_obj MAX_OBJ
#endif
#ifdef num_of_all_obj
	/* hardware obj array, use max value for default*/
	#ifndef NUM_OF_HWOBJ
	#define  NUM_OF_HWOBJ MAX_OBJ
	#endif
	/* soft timer obj array, use max value for default*/
	#ifndef	NUM_OF_TIMER
	#define  NUM_OF_TIMER MAX_OBJ
	#endif
#endif
/*----------------------------------------------------------------------*/
/*id macro*/
#define idof_obj						OBJ_ID.object_id
#define class_of_obj					OBJ_ID.object_class

/*status macro*/
#define obj_status						OBJ_STATUS.byte
#define obj_state						OBJ_STATUS.soft.state
#define obj_event						OBJ_STATUS.soft.event
#define extended_value					OBJ_STATUS.soft.ext

/*value macro*/
#define obj_value						OBJ_VALUE.value
#define obj_def_value					OBJ_VALUE.def.default_value
#define obj_ext_value					OBJ_VALUE.extended_value.extended_value
#define obj_text_page_content			OBJ_VALUE.info_block.page_content
#define obj_text_page_number			OBJ_VALUE.info_block.page_number		
#define obj_text_num_of_pages			OBJ_VALUE.info_block.page_content

/*bind macro */
#define obj_type_soft					OBJ_BIND.soft
#define obj_type_hardware				OBJ_BIND.hardware
#define obj_type_timer					OBJ_BIND.timer

/*obj access*/
#define this_obj(_obj)					(OBJ_MODEL_CLASS.objDefault + _obj)
#define OBJ(obj)						OBJ_MODEL_CLASS.OBJ_AREA.OBJ

/*obj sync*/

#define this_obj_state(obj_id)			this_obj(obj_id)->obj_state
#define obj_state_on(obj_id)			this_obj(obj_id)->obj_state = TRUE
#define obj_state_off(obj_id)			this_obj(obj_id)->obj_state = FALSE
#define state_of_obj(obj)				this_obj(obj)->obj_state
#define value_of_obj(obj)				this_obj(obj)->obj_def_value
#define obj_set_state(obj,state)		this_obj(obj)->obj_state = state
#define obj_set_value(obj,state)		this_obj(obj)->obj_value = value

#define OBJ_Event(id)	OBJ_MODEL_CLASS.OBJ_AREA.OBJ[id].OBJ_SYNC.status.byte |= obj_event_mask

/*----------------------------------------------------------------------*/

/*common functions*/
void obj_model_init( void );
void obj_bind( OBJ_INIT_TypeDef* _model_init_,int _model_size_ );
void obj_soft_create( int obj_id, int obj_class );
void obj_hardware_create( int obj_id, int obj_class,int hwobj );
void obj_timer_create( int obj_id, int obj_class,uint16_t delay,void (*handler_pointer)(OBJ_STRUCT_TypeDef*) );
void obj_sync( OBJ_STRUCT_TypeDef *instance );
void obj_model_thread( void );
void obj_event_fnct( int obj_id );

/*serial port functions*/
void sp_all_obj_sync( void );
void sp_mes_receive( USART_FRAME_TypeDef *mes );


extern OBJ_MODEL_CLASS_TypeDef OBJ_MODEL_CLASS;
/*----------------------------------------------------------------------*/

/*----------------------------------------------------------------------*/


#define obj_event_mask					0x02
#define obj_status_mask					0x01
/*---------------------------------------------*/
#define USART1_DEFAULT_BUF_SIZE 		LEN_USART_MSG_OBJ

#define START_BYTE_0	0xAA
#define START_BYTE_1	0x55
#define STOP_BYTE		0xFF
/*-----------------------------------------------*/
#define NSD			0			// not used option	
/*---------------------------------------------*/
#define tick_1ms	1
#define tick_5ms	5
#define tick_10ms	10
#define tick_25ms	25
#define tick_50ms	50
#define tick_100ms	100
#define tick_250ms	250
#define tick_500ms	500
#define tick_1s		1000
/*-----------------------------------------------*/
#define can_obj_mask	0x000000FF
#define can_id_mask		0x00003F00
#define can_net_mask	0x0000C000
/*-----------------------------------------------*/
#define board_power	board_state.bit.power
/*-----------------------------------------------*/
#define TRUE    1
#define FALSE   0
/*-----------------------------------------------
**************model operation mode**************
-----------------------------------------------*/
#define APP_MODE	1
#define BOOT_MODE	2

#ifdef USE_RTOS
	#include "RTOS.h"
#endif
/*-----------------------------------------------
***************COMMON VARIABLES******************
-----------------------------------------------*/
extern BOARD_STATE	board_state;
extern uint32_t num_of_obj;

#if RTOS_USAGE == TRUE
	extern OBJ_MODEL_PRIORITY task_priority;
#endif


#if USART_COM_ENABLE == TRUE	
	/* data array for usart obj receive */
	extern uint8_t usart_data_receive_array[USART1_DEFAULT_BUF_SIZE];
	#if RTOS_USAGE == TRUE
		/*mutex  to perform currect usart transmit */
		extern xSemaphoreHandle xMutex_USART_BUSY;
		/*queue of messages from usart module*/
		extern xQueueHandle usart_receive_buffer;
	#endif
	/*usart data byte counter */
	extern uint8_t usart_irq_counter;
#endif

#if CAN_COM_ENABLE == TRUE
	extern xQueueHandle can_receive_buffer;
	extern xQueueHandle can_transmit_buffer;
	CAN_OBJ_FRAME can_obj_create_message (int obj_id);
	uint32_t can_queue_obj_fill(CAN_OBJ_FRAME message);
	void can_obj_send_routine(uint32_t tick);
#endif

/*-----------------------------------------------
*********OBJ MODEL FUNCTION PROTOTYPES***********
-----------------------------------------------*/
/*init obj model tasks*/
void OBJ_task_init(OBJ_MODEL_PRIORITY *task_priority,int tick_update_rate);

/*RTOS tasks*/
void _task__OBJ_model_thread (void *pvParameters);
void _task__OBJ_data_rx (void *pvParameters);
/*-----------------------------------------------
****DEVICE SPECIFIC COMMUNICATION FUNCTIONS*****
-----------------------------------------------*/
/*usart transfer , board specific */
void send_usart_message(uint8_t *message,uint32_t buf_size);
/*-----------------------------------------------
***********REDEFINABLE FUNCTIONS*****************
-----------------------------------------------*/
/*hardware event handler, board special*/
void HWOBJ_Event(int obj_id);
/*empty handler (can be changed)*/
void Dummy_Handler(OBJ_STRUCT_TypeDef *obj);
/*obj model setup, config usart update rate, config object initial state */
void obj_model_setup(void);
/*obj model loop */
void obj_model_task(int tick);
/*-----------------------------------------------*/

#endif
