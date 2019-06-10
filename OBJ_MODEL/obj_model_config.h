#ifndef OBJ_CONFIG_H_
#define	OBJ_CONFIG_H_

#define TRUE    1
#define FALSE   0

#define USART_MODE  0
#define CAN_MODE    1

#define DEBUG_MODE	TRUE

#define HARDWARE_OBJECT TRUE
#define USART_DATA_FAST	TRUE

#define	num_of_all_obj		  255

#define MES_BUF_SIZE		20

#ifdef HARDWARE_OBJECT
    #define NUM_OF_HWOBJ    20
#endif
#define obj_limit	40

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

