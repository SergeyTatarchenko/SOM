/*************************************************************************
* File Name          : crc_module.h
* Author             : Tatarchenko S.
* Version            : v 1.0
* Description        : crc calculation unit
*************************************************************************/
#include "crc_module.h"

/*----------------------------------------------------------------------
crc calculation function
(used with CRC_UNIT define)
-----------------------------------------------------------------------*/
#ifdef CRC_UNIT
uint16_t sp_calc_crc( USART_FRAME_TypeDef *mes ) 
{
	uint16_t _CRC = 0;
	
	return _CRC;
}
#endif
/*----------------------------------------------------------------------
default hash function (sum of bytes of the message)
-----------------------------------------------------------------------*/
uint16_t default_calc_crc( void *mes, int size) 

{
	int i = 0;
	uint16_t _CRC = 0;
	
//	for(i = LEN_START; i < LEN_USART_MSG_OBJ - (LEN_CRC + LEN_STOP); i++)
//	{
//		_CRC += mes->byte[i];
//	}
	return _CRC;
}


#if (TypeCRC == CRC_Calc)

  void Clac_CRC_byte(uint8_t bMess, uint16_t *CRC_val)
  {
    for (uint8_t i = BIT_SYS; i > 0; i--) 
    {
      uint8_t mix = ((*CRC_val) ^ bMess) & MSB;
      *CRC_val <<= 1;
      if (mix) *CRC_val ^= CRC_POLYNOM;
      bMess <<= 1;
    }
  }
#elif (TypeCRC == CRC_Table)
  
  void Clac_CRC_byte(uint8_t bMess, uint16_t *CRC_val)
  {
		uint16_t icrc = bMess ^ (*CRC_val);

        if(fBit == LSB)
        {
            *CRC_val = TableCRC15_CAN[icrc] ^ ((*CRC_val) >> 8);
        }else if(fBit == MSB)
        {
            *CRC_val = TableCRC15_CAN[icrc] ^ ((*CRC_val) << 8);
        }
  }
#elif (TypeCRC == CRC_Summ)
  
  void Clac_CRC_byte(uint8_t bMess, uint16_t *CRC_val)
  {
	  *CRC_val += bMess;
  }
#endif
  
///**********************************************************************************/
//// Calc_CRC_arr
///**********************************************************************************/
//uint16_t Calc_CRC_arr(uint8_t *pMess, uint8_t len)
//{
//	uint16_t crc_val = 0;
//	uint8_t i = 0;
//	
//	while(i++ < len)
//	{
//		Clac_CRC_byte(*(pMess + i), &crc_val)
//	}
//	
//	return crc_val;
//}