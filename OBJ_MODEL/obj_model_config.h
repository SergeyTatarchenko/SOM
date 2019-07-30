
#ifndef OBJ_CONFIG_H_
#define	OBJ_CONFIG_H_

#define TRUE    1
#define FALSE   0

#define USART_MODE	TRUE
#define CAN_MODE    FALSE

#define DEBUG_MODE	TRUE

#define HARDWARE_OBJECT TRUE
#define USART_DATA_FAST	TRUE
#define OBJECT_TIMER	TRUE

#define	num_of_all_obj		30
#define MES_BUF_SIZE		20


#if HARDWARE_OBJECT == TRUE
	#define NUM_OF_HWOBJ    20
#endif

#if OBJECT_TIMER == TRUE
	#define NUM_OF_TIMER    20
#endif

#ifndef TARGET
	#include "DEFAULT.h"
#endif

/*-------------------------------------------------
                  FreeRTOS 
-------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h" 
/*-------------------------------------------------
                !board special!
-------------------------------------------------*/
/*     network and board description             */
#ifdef	TARGET

	#if	TARGET == 0
		#include "TEST.h"
	#elif TARGET == 72
		#include "B72.h"
	#endif

#endif

#endif

