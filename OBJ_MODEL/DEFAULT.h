/*------------------------------------------------------------------------
* File Name          : DEFAULT.h
* Author             : 
* Version            : v 1.1.0
* Description        : default example of model init 
------------------------------------------------------------------------*/

#ifndef DEFAULT_H_
#define	DEFAULT_H_

/*----------------------------------------------------------------------*/
#define	ID_NETWORK      0x01
#define	ID_DEVICE       0x01
#define ID_REMOTE_CNTRL 0x00
#define	FLAG_RX_ALL     0xFF

#define IND_obj_NULL      0

#define obj_STATUS       (IND_obj_NULL + 1)
#define obj_TEST0        (IND_obj_NULL + 2)
#define obj_TEST1        (IND_obj_NULL + 3)
#define obj_TEST2        (IND_obj_NULL + 4)
#define obj_TEST3        (IND_obj_NULL + 5)
#define obj_TEST4        (IND_obj_NULL + 6)
#define obj_TEST5        (IND_obj_NULL + 7)

/*
----------------------------------------------------------------------------------------------------------;
           index          |     class        |   type    | hw_snap   |    delay    |   Handler         
----------------------------------------------------------------------------------------------------------;
*/
#define _obj_cofig_	\
{     obj_STATUS          ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }, \
{     obj_TEST0           ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }, \
{     obj_TEST1           ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }, \
{     obj_TEST2           ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }, \
{     obj_TEST3           ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }, \
{     obj_TEST4           ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }, \
{     obj_TEST5           ,IND_obj_CAS       ,obj_soft   ,   NSD     ,  NSD        ,NSD                  }

/*--------------------------------------------------------------------------------------------------*/

#endif
