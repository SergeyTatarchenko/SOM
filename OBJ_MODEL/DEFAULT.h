#ifndef DEFAULT_H_
#define	DEFAULT_H_
/*------------------------------------------------------------------------
* File Name          : DEFAULT.h
* Author             : 
* Version            : v 1.1.0
* Description        : default example of model init 
------------------------------------------------------------------------*/
#include "OBJ_MODEL.h"
/*----------------------------------------------------------------------*/
#define	IND_obj_NULL			0x00
#define	IND_obj_END				0xFF
/*----------------------------------------------------------------------*/
#define obj_STATUS			  (IND_obj_NULL + 1)
#define obj_TEST			  (IND_obj_NULL + 2)
/*----------------------------------------------------------------------*/
/*
v 0.6
1) create object name  example   " #define obj_name	(IND_obj_NULL + x)"
2) create object init like "obj_index,class,snap,delay,handler in  _obj_cofig_ define	
*/

/*
----------------------------------------------------------------------------------------------------------;
           index          |     class        |   type    | hw_snap   |    delay    |   Handler         
----------------------------------------------------------------------------------------------------------;
*/
#define _obj_cofig_	\
{     obj_STATUS          ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }, \
{     obj_TEST            ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }

/*--------------------------------------------------------------------------------------------------*/

#endif
