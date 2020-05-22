/************************************************************************
* File Name          : model_config.h
* Author             : 
* Version            : v 1.0.0
* Description        : template for model configuration
*************************************************************************/

#ifndef OBJ_CONFIG_H_
#define	OBJ_CONFIG_H_

/*------------------------------------------------------------------------
*************************model operation mode*****************************
------------------------------------------------------------------------*/
#define TRUE    1
#define FALSE   0

#define APP_MODE    1
#define BOOT_MODE   2


#define SOM_MODE	APP_MODE             /*APP_MODE or BOOT_MODE selection*/

#define USE_RTOS	TRUE        /*enable to use FreeRTOS features in model*/

#define USE_SERIAL_PORT TRUE        /*enable to use serial port in model*/

#define USE_CAN_BUS TRUE                /*enable to use CAN bus in model*/

#define USE_HWOBJ TRUE         /*enable to use hardware objects in model*/

#define USE_TIMERS TRUE         /*enable to use software timers in model*/

//#define TARGET 0                            /*define model id in network*/

#ifndef TARGET
#include "DEFAULT.h"
#endif

#ifdef USE_RTOS
#include "RTOS.h"
#endif

#if USE_TIMERS == TRUE
#ifndef USE_RTOS
#error "software timers used only with FreeRTOS include"
#endif
#endif

/*include header file with objects description  _obj_cofig_ DEFINE */

#endif
/*------------------------end of file----------------------------------*/
