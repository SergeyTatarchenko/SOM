#ifndef TEST_H_
#define	TEST_H_

#include "OBJ_MODEL.h"

/*-------------------------------------------------*/

#define	IND_obj_NULL			0x00
#define	IND_obj_END				0xFF

#define obj_STATUS			  (IND_obj_NULL + 1)	

#define	ID_NETWORK			0x00
#define	ID_DEVICE			0xFF
#define ID_REMOTE_CNTRL		0x02
#define	FLAG_RX_ALL			0xFF

/*
v 0.3
1) create object name  example   " #define obj_name	(IND_obj_NULL + x)"
2) create object init like "obj_name_init  obj_index,class,snap,handler	
3) add 	object init to 	_obj_cofig_	
*/
/*----------------------------------------------------------------------------------------------------\
           name           |      index          |   class     | type     | hw_snap  |   Handler       | 
\----------------------------------------------------------------------------------------------------*/

#define _obj_STATUS_init       obj_STATUS           ,IND_obj_CAS  ,obj_soft  ,   NULL   ,NULL

#define _obj_cofig_	{_obj_STATUS_init}
					

/*--------------------------------------------------------------------------------------*/
			
/*-------------------------------------------------*/
/*obj handlers*/
#endif
