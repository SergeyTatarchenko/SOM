/*************************************************
* File Name          : OBJ_MODEL.h
* Author             : Tatarchenko S.
* Version            : v 1.2
* Description        : header for OBJ_MODEL.c 
*************************************************/
#ifndef OBJ_DATA_H_
#define	OBJ_DATA_H_

/*-----------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/*----------------------------------------------*/
#include "string.h"
#include "stdint.h"
/*-----------------------------------------------*/
/*-----------------------------------------------*/
/*-----------------------------------------------*/
/*------------object description-----------------*/
/*-----------------------------------------------*/
#define LEN_NETW	1
#define LEN_ID		1
#define	LEN_INDEX	2
#define LEN_DATA	8
#define	LEN_CRC		2
#define LEN_HEAD_SIZE			(LEN_NETW + LEN_ID)
#define LEN_OBJ					(LEN_INDEX + LEN_DATA + LEN_CRC)
#define	LEN_USART_MSG_OBJ		(LEN_NETW + LEN_ID + LEN_INDEX + LEN_DATA + LEN_CRC)
/*-----------------------------------------------*/
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
					unsigned visible  : 1;
					unsigned hardware  : 1;
				}bit;
			}control_byte;
			uint16_t HW_adress;
			uint8_t rezerv[3];
			uint16_t value;
		}default_field;
		/* extended data field for data transmit*/
		struct{
			uint8_t data[8];
		}data_field;
	}obj_field;	
}OBJ_STRUCT;
#pragma pack(pop)

/*incompressible struct*/
typedef struct {
	uint8_t id;
	uint8_t obj_class;
	uint8_t obj_type;
	uint16_t HW_adress;
	void (*handler_pointer)(OBJ_STRUCT*);
	/*add initial state and ...*/
}obj_init_struct;


#pragma pack(push,1)
typedef union{
	uint8_t status;
	struct{
		unsigned power:1;
		unsigned mode:1;
		unsigned hwobj:1;
		unsigned debug:1;
		unsigned rez4:1;
		unsigned rez5:1;
		unsigned rez6:1;
		unsigned rez7:1;
	}bit;
}BOARD_STATE;
#pragma pack(pop)

/*                 object type                   */
#define IND_obj_COM				4	/*data command*/
#define IND_obj_CAS				3	/*control and status*/
#define IND_obj_CWS				2	/*control without status*/
#define IND_obj_SWC				1	/*status without control*/	
/*---------------------------------------------*/
#define event_mask				0x02
#define state_mask				0x01
#define hardware_mask			0x80
/*---------------------------------------------*/
#define typeof_obj				id[1]
#define status_field 			obj_field.default_field.control_byte.byte
#define obj_event				obj_field.default_field.control_byte.bit.event
#define obj_state				obj_field.default_field.control_byte.bit.state
#define obj_value				obj_field.default_field.value
#define hardware_adress			obj_field.default_field.HW_adress
#define obj_visible				obj_field.default_field.control_byte.bit.visible
#define obj_hardware			obj_field.default_field.control_byte.bit.hardware
#define obj_data				obj_field.data_field.data
/*---------------------------------------------*/
#define obj_soft	0
#define obj_hard	1
/*---------------------------------------------*/
#define this_obj(obj_id)				(objDefault + obj_id)
#define obj_set_visible(obj_id,state)	this_obj(obj_id)->obj_visible = state
#define obj_update(obj_id)				obj_handlers[obj_id](this_obj(obj_id))												
#define obj_state_on(obj_id)			this_obj(obj_id)->obj_state = TRUE;	\
										OBJ_Event(obj_id)
#define obj_state_off(obj_id)			this_obj(obj_id)->obj_state = FALSE; \
										OBJ_Event(obj_id)


/*-----------------------------------------------*/
/*-----------struct for USART frame--------------*/
/*-----------------------------------------------*/
#pragma pack(push,1)
typedef union{
	struct{
        uint8_t id_netw;
        uint8_t id_modul;
        OBJ_STRUCT object;
        uint16_t crc;
    }d_struct;
    uint8_t byte[LEN_USART_MSG_OBJ];
}USART_FRAME;
#pragma pack(pop)


#define USART1_DEFAULT_BUF_SIZE 14
	
#ifdef	LEN_USART_MSG_OBJ
	#undef USART1_DEFAULT_BUF_SIZE
	#define USART1_DEFAULT_BUF_SIZE LEN_USART_MSG_OBJ
#endif

#define USART_STREAM_SIZE	(USART1_DEFAULT_BUF_SIZE*256)

#define USART_DATA_TYPE1	1
#define USART_DATA_TYPE2	2

/*-----------------------------------------------*/
/*         system arrays for USART               */
/*-----------------------------------------------*/

/* data array for usart obj transfer */
extern uint8_t	usart_data_transmit_array[USART1_DEFAULT_BUF_SIZE];
extern uint8_t	usart_data_stream[USART_STREAM_SIZE];
/* data array for usart obj receive */
extern uint8_t usart_data_receive_array[USART1_DEFAULT_BUF_SIZE];
/*mutex  to perform currect usart transmit */
extern xSemaphoreHandle xMutex_USART_BUSY;
/*queue of messages from usart module*/
extern xQueueHandle usart_receive_buffer;
/*usart data byte counter */
extern uint8_t usart_irq_counter;

/*-----------------------------------------------*/
/*-----------------------------------------------*/
/*           struct for CAN frame                */
/*-----------------------------------------------*/
/*-----------------------------------------------*/
/*         system arrays for CAN                 */
/*-----------------------------------------------*/
/*-----------------------------------------------*/


/*-----------------------------------------------*/
/*--------------Common variables-----------------*/
/*-----------------------------------------------*/
#include "obj_model_config.h"
/*-----------------------------------------------*/
/*pointer to memory space of objects*/
extern OBJ_STRUCT *objDefault;
/*array of object handlers*/
extern void ((*obj_handlers[num_of_all_obj +1]))(OBJ_STRUCT*);

extern BOARD_STATE	board_state;

extern uint32_t num_of_obj;

#ifndef HARDWARE_OBJECT
	#error "HARDWARE_OBJECT is undefined"
#endif

#ifndef USART_DATA_FAST
	#error "USART_DATA_FAST is undefined"
#endif

#if	HARDWARE_OBJECT == TRUE
	extern OBJ_STRUCT *HW_OBJ[NUM_OF_HWOBJ];
#endif	

#if USART_DATA_FAST == TRUE
	extern uint8_t USART_DATA[sizeof(USART_FRAME)*num_of_all_obj];	
#endif
void Dummy_Handler(OBJ_STRUCT *obj);
/*-----------------------------------------------*/
/*-----------------------------------------------*/
/*----------Common functions prototypes----------*/
/*-----------------------------------------------*/

/*init obj model*/
void OBJ_Init(void);
/*create object*/
OBJ_STRUCT* Obj_Create(int obj_id, int obj_type);
/*create hardware object, return pointer to obj */
OBJ_STRUCT* HWObj_Create(int obj_id, int obj_type,int hwobj);
/*object event*/
extern void OBJ_Event(int obj_id);
/*set obj state*/
extern void OBJ_SetState(int obj_id,int state);
/*hardware event handler, board special*/
extern void HWOBJ_Event(int obj_id);
/*update this obj */
extern void OBJ_Upd_USART(OBJ_STRUCT *obj);
/*update all obj */
extern void Upd_All_OBJ_USART(void);
/* FAST!!! update all obj */
extern void FAST_Upd_All_OBJ_USART(void);
/*receive object data from message*/
extern void Rx_OBJ_Data(USART_FRAME *mes);
/*check control sum of receive data*/
extern uint8_t Check_CRC(USART_FRAME *Rx_obj_c);

/*-----------------------------------------------*/
extern void obj_snap(obj_init_struct* _model_init_,int _model_size_);

/*usart transfer , board specific */
__weak void send_usart_message(uint8_t *message,uint32_t buf_size);

/*-----------------------------------------------*/
#endif
