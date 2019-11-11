/*************************************************
* File Name          : OBJ_MODEL.h
* Author             : Tatarchenko S.
* Version            : v 1.5.3
* Description        : header for OBJ_MODEL.c 
*************************************************/
#ifndef OBJ_DATA_H_
#define	OBJ_DATA_H_
/*-----------------------------------------------*/
#include "stdint.h"
#include "string.h"
/*-----------------------------------------------*/
#define NSD			0			// not used option	
/*-----------------------------------------------*/
/*------------object description-----------------*/
/*-----------------------------------------------*/
#define LEN_START	2
#define LEN_NETW	1
#define LEN_ID		1
#define	LEN_INDEX	2
#define LEN_DATA	8
#define	LEN_CRC		2
#define	LEN_STOP	1
#define LEN_HEAD_SIZE			(LEN_START + LEN_NETW + LEN_ID )
#define LEN_OBJ					(LEN_INDEX + LEN_DATA + LEN_CRC)
#define	LEN_USART_MSG_OBJ		(LEN_START + LEN_NETW + LEN_ID + LEN_INDEX + LEN_DATA + LEN_CRC + LEN_STOP)
/*-----------------------------------------------*/

/* enums to describe types and priorities of objects*/
typedef enum{
	
	/*status without control, low priority*/
	IND_obj_SWC = 1,
	/*control without control, medium priority*/
	IND_obj_CWS = 2,
	/*control with status, high priority*/
	IND_obj_CAS = 3,
	/*object with extended data field, high priority*/
	IND_obj_COM = 4
	
}OBJECT_CLASS;

typedef enum{
	/*software object, default object*/	
	obj_soft =  0,
	/*hardware object, use special hanlder*/
	obj_hard  = 1,
	/*software timer (with RTOS_USAGE only)*/
	obj_timer = 2,
	/*info object,use extended data field, contains text*/
	obj_text =  3
}OBJECT_TYPE;

typedef struct{
	
	int system_priority;
	int tx_priority;
	int rx_priority;
	int user_priority;
	
	unsigned short stack_tx_rx;
	unsigned short stack_user;
	int tick_update_rate;

}OBJ_MODEL_PRIORITY;
/*-----------------------------------------------
*********OBJECT STRUCTURE DESCRIPTION***********
-----------------------------------------------*/
#pragma pack(push,1)
typedef	struct{
	/* ID 2 byte*/
	uint8_t id[2];
	/* Data 8 byte*/
	union {		
		/*default obj field*/
		struct{
			union{
				uint8_t byte;
				struct{
					unsigned state : 1;
					unsigned event : 1;
					unsigned rez2  : 1;
					unsigned rez3  : 1;
					unsigned rez4  : 1;
					unsigned rez5  : 1;
					unsigned rez6  : 1;
					unsigned hardware  : 1;
				}bit;
			}control_byte;
			union{
				/* max 255 hwobj */
				uint8_t HW_adress;
				/*max 255 soft timers */
				uint8_t TimerID;
			}obj_type;
			/*rezerv*/
			uint8_t rezerv[4];
			/*value field*/
			uint16_t value;
		}default_field;
		/* extended data field for data transmit*/
		struct{
			union{
				uint8_t data[8];
				struct{
					uint32_t dL;
					uint32_t dH;
				}dWord;
			}extended_field;
		}data_field;
	}obj_field;	
}OBJ_STRUCT;
#pragma pack(pop)
/*-----------------------------------------------
**STRUCTURE FOR INITIALIZING OBJECTS IN MEMORY**
-----------------------------------------------*/
#pragma pack(push,1)
typedef struct {
	uint8_t id;
	OBJECT_CLASS obj_class;
	OBJECT_TYPE obj_type;
	uint8_t HW_adress;
	uint16_t delay;
	void (*handler_pointer)(OBJ_STRUCT*);
}obj_init_struct;
#pragma pack(pop)

/*-----------------------------------------------
*********STRUCT FOR USART TRANSMIT**************
-----------------------------------------------*/
#pragma pack(push,1)
typedef union{
	struct{
		uint8_t start_seq[2];  
        uint8_t id_netw;
        uint8_t id_modul;
        OBJ_STRUCT object;
        uint16_t crc;
		uint8_t stop_seq;
    }d_struct;
    uint8_t byte[LEN_USART_MSG_OBJ];
}USART_FRAME;
#pragma pack(pop)
/*-----------------------------------------------
************STRUCT FOR CAN TRANSMIT**************
-----------------------------------------------*/
typedef struct{
	uint32_t id;	
	uint8_t  data[8];	
	uint8_t  len;  				
}CAN_OBJ_FRAME;  


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

#include "obj_model_config.h"
/*---------------------------------------------*/
#define idof_obj						id[0]
#define typeof_obj						id[1]
#define status_field 					obj_field.default_field.control_byte.byte
#define obj_event						obj_field.default_field.control_byte.bit.event
#define obj_state						obj_field.default_field.control_byte.bit.state
#define obj_value						obj_field.default_field.value
#define hardware_adress					obj_field.default_field.obj_type.HW_adress
#define timer_adress					obj_field.default_field.obj_type.TimerID
#define obj_hardware					obj_field.default_field.control_byte.bit.hardware
#define obj_data						obj_field.data_field.data

#define dWordL							obj_field.data_field.extended_field.dWord.dL
#define dWordH							obj_field.data_field.extended_field.dWord.dH
/*---------------------------------------------*/
#define this_obj(obj_id)				(objDefault + obj_id)
#define this_obj_state(obj_id)			this_obj(obj_id)->obj_state
#define obj_update(obj_id)				obj_handlers[obj_id](this_obj(obj_id))
#define obj_state_on(obj_id)			this_obj(obj_id)->obj_state = TRUE;	\
										OBJ_Event(obj_id)
#define obj_state_off(obj_id)			this_obj(obj_id)->obj_state = FALSE; \
										OBJ_Event(obj_id)										
#define state_of_obj(obj)				this_obj(obj)->obj_state
#define value_of_obj(obj)				this_obj(obj)->obj_value
#define obj_set_state(obj,state)		this_obj(obj)->obj_state = state
#define obj_set_value(obj,state)		this_obj(obj)->obj_value = value
/*---------------------------------------------*/
#define USART1_DEFAULT_BUF_SIZE 		LEN_USART_MSG_OBJ
#define USART_STREAM_SIZE				(USART1_DEFAULT_BUF_SIZE*num_of_all_obj)
#define USART_DATA_TYPE1				1
#define USART_DATA_TYPE2				2

#define START_BYTE_0	0xAA
#define START_BYTE_1	0x55
#define STOP_BYTE		0xFF
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

#ifndef USART_DATA_FAST
	#error "USART_DATA_FAST is undefined"
#endif

#if RTOS_USAGE == TRUE
	#include "RTOS.h"
#endif

/*-----------------------------------------------*/

#if USART_COM_ENABLE == TRUE
	/* data array for usart obj transfer */
	extern uint8_t	usart_data_transmit_array[USART1_DEFAULT_BUF_SIZE];
	extern uint8_t	usart_data_stream[USART_STREAM_SIZE];
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
***************COMMON VARIABLES******************
-----------------------------------------------*/

extern OBJ_STRUCT *objDefault;
extern BOARD_STATE	board_state;
extern uint32_t num_of_obj;

#if RTOS_USAGE == TRUE
	extern OBJ_MODEL_PRIORITY task_priority;
#endif
extern void ((*obj_handlers[num_of_all_obj+1]))(void*);

#if	HARDWARE_OBJECT == TRUE
	extern OBJ_STRUCT *HW_OBJ[NUM_OF_HWOBJ];
#endif	

#if USART_DATA_FAST == TRUE
	extern uint8_t USART_DATA[sizeof(USART_FRAME)*num_of_all_obj];	
#endif

/*-----------------------------------------------
*********OBJ MODEL FUNCTION PROTOTYPES***********
-----------------------------------------------*/
/*init obj model memory*/
void OBJ_Init(void);
/*object init without task and queue (mimimal init!!!)*/
void SOM_Init(void);
/*init obj model tasks*/
void OBJ_task_init(OBJ_MODEL_PRIORITY *task_priority,int tick_update_rate);
/*object memory binding*/
void obj_snap(obj_init_struct* _model_init_,int _model_size_);
/*create object*/
OBJ_STRUCT* Obj_Create(int obj_id, int obj_type);
/*create hardware object, return pointer to obj */
OBJ_STRUCT* HWObj_Create(int obj_id, int obj_type,int hwobj);
/* create timer */
OBJ_STRUCT* Timer_Create(int obj_id, int obj_type,uint16_t delay,void (*handler_pointer)(OBJ_STRUCT*));
/*object event*/
void OBJ_Event(int obj_id);
/*set obj state*/
void OBJ_SetState(int obj_id,int state);
/*test*/
void set_all_obj_off(void);
/* FAST!!! update all obj */
void FAST_Upd_All_OBJ_USART(void);
/*receive object data from message*/
void Rx_OBJ_Data(USART_FRAME *mes);
/*check control sum of receive data*/
uint8_t Check_CRC(USART_FRAME *Rx_obj_c);

/*RTOS tasks*/
void _task__OBJ_model_thread (void *pvParameters);
void _task__OBJ_data_rx (void *pvParameters);
void _task__OBJ_data_tx(void *pvParameters);
/*-----------------------------------------------
****DEVICE SPECIFIC COMMUNICATION FUNCTIONS*****
-----------------------------------------------*/
/*usart transfer , board specific */
void send_usart_message(uint8_t *message,uint32_t buf_size);
/*CAN transfer, board specific*/
void send_can_message(CAN_OBJ_FRAME message);
/*-----------------------------------------------
***********REDEFINABLE FUNCTIONS*****************
-----------------------------------------------*/
/*hardware event handler, board special*/
void HWOBJ_Event(int obj_id);
/*empty handler (can be changed)*/
void Dummy_Handler(OBJ_STRUCT *obj);
/*obj model setup, config usart update rate, config object initial state */
void obj_model_setup(void);
/*obj model loop */
void obj_model_task(int tick);
/*-----------------------------------------------*/
#endif
