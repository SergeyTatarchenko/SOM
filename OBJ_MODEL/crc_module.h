/*************************************************************************
* File Name          : crc_module.h
* Author             : Tatarchenko S.
* Version            : v 1.0
* Description        : header for crc_module.c 
*************************************************************************/
#ifndef CRC_MODULE
#define CRC_MODULE
/*----------------------------------------------------------------------*/
#include "OBJ_MODEL.h"
/*----------------------------------------------------------------------*/
typedef enum{
	NO_CRC,
	DEFAULT,
	CRC15_CAN
}CRC_TYPE;
/*----------------------------------------------------------------------*/

#define CRC_Summ		0x0
#define CRC_Calc		0x1
#define CRC_Table		0x2

#define TypeCRC       	CRC_Table

#if (TypeCRC != CRC_Summ)
	#define MSB           	0x80
	#define LSB           	0x01
	#define	fBit			MSB
	#define BIT_SYS       	8
	#define CRC_POLYNOM   	0x4599
#endif

#if (TypeCRC == CRC_Calc)
  
  void Clac_CRC_byte(uint8_t bMess, uint16_t *CRC_val);	// Byte message Calc CRC

#endif

uint16_t Calc_CRC_arr(uint8_t *pMess, uint8_t len);	// Array message Calc CRC
/*----------------------------------------------------------------------*/
const unsigned short TableCRC15_CAN [256] = {
0x0000, 0x4599, 0xCEAB, 0x8B32, 0xD8CF, 0x9D56, 0x1664, 0x53FD, 
0xF407, 0xB19E, 0x3AAC, 0x7F35, 0x2CC8, 0x6951, 0xE263, 0xA7FA, 
0xE80E, 0xAD97, 0x26A5, 0x633C, 0x30C1, 0x7558, 0xFE6A, 0xBBF3, 
0x1C09, 0x5990, 0xD2A2, 0x973B, 0xC4C6, 0x815F, 0x0A6D, 0x4FF4, 
0xD01C, 0x9585, 0x1EB7, 0x5B2E, 0x08D3, 0x4D4A, 0xC678, 0x83E1, 
0x241B, 0x6182, 0xEAB0, 0xAF29, 0xFCD4, 0xB94D, 0x327F, 0x77E6, 
0x3812, 0x7D8B, 0xF6B9, 0xB320, 0xE0DD, 0xA544, 0x2E76, 0x6BEF, 
0xCC15, 0x898C, 0x02BE, 0x4727, 0x14DA, 0x5143, 0xDA71, 0x9FE8, 
0xA038, 0xE5A1, 0x6E93, 0x2B0A, 0x78F7, 0x3D6E, 0xB65C, 0xF3C5, 
0x543F, 0x11A6, 0x9A94, 0xDF0D, 0x8CF0, 0xC969, 0x425B, 0x07C2, 
0x4836, 0x0DAF, 0x869D, 0xC304, 0x90F9, 0xD560, 0x5E52, 0x1BCB, 
0xBC31, 0xF9A8, 0x729A, 0x3703, 0x64FE, 0x2167, 0xAA55, 0xEFCC, 
0x7024, 0x35BD, 0xBE8F, 0xFB16, 0xA8EB, 0xED72, 0x6640, 0x23D9, 
0x8423, 0xC1BA, 0x4A88, 0x0F11, 0x5CEC, 0x1975, 0x9247, 0xD7DE, 
0x982A, 0xDDB3, 0x5681, 0x1318, 0x40E5, 0x057C, 0x8E4E, 0xCBD7, 
0x6C2D, 0x29B4, 0xA286, 0xE71F, 0xB4E2, 0xF17B, 0x7A49, 0x3FD0, 
0x4070, 0x05E9, 0x8EDB, 0xCB42, 0x98BF, 0xDD26, 0x5614, 0x138D, 
0xB477, 0xF1EE, 0x7ADC, 0x3F45, 0x6CB8, 0x2921, 0xA213, 0xE78A, 
0xA87E, 0xEDE7, 0x66D5, 0x234C, 0x70B1, 0x3528, 0xBE1A, 0xFB83, 
0x5C79, 0x19E0, 0x92D2, 0xD74B, 0x84B6, 0xC12F, 0x4A1D, 0x0F84, 
0x906C, 0xD5F5, 0x5EC7, 0x1B5E, 0x48A3, 0x0D3A, 0x8608, 0xC391, 
0x646B, 0x21F2, 0xAAC0, 0xEF59, 0xBCA4, 0xF93D, 0x720F, 0x3796, 
0x7862, 0x3DFB, 0xB6C9, 0xF350, 0xA0AD, 0xE534, 0x6E06, 0x2B9F, 
0x8C65, 0xC9FC, 0x42CE, 0x0757, 0x54AA, 0x1133, 0x9A01, 0xDF98, 
0xE048, 0xA5D1, 0x2EE3, 0x6B7A, 0x3887, 0x7D1E, 0xF62C, 0xB3B5, 
0x144F, 0x51D6, 0xDAE4, 0x9F7D, 0xCC80, 0x8919, 0x022B, 0x47B2, 
0x0846, 0x4DDF, 0xC6ED, 0x8374, 0xD089, 0x9510, 0x1E22, 0x5BBB, 
0xFC41, 0xB9D8, 0x32EA, 0x7773, 0x248E, 0x6117, 0xEA25, 0xAFBC, 
0x3054, 0x75CD, 0xFEFF, 0xBB66, 0xE89B, 0xAD02, 0x2630, 0x63A9, 
0xC453, 0x81CA, 0x0AF8, 0x4F61, 0x1C9C, 0x5905, 0xD237, 0x97AE, 
0xD85A, 0x9DC3, 0x16F1, 0x5368, 0x0095, 0x450C, 0xCE3E, 0x8BA7, 
0x2C5D, 0x69C4, 0xE2F6, 0xA76F, 0xF492, 0xB10B, 0x3A39, 0x7FA0, 
};
#endif
/*------------------------end of file----------------------------------*/
