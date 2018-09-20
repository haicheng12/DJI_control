#ifndef _CAN_h_
#define _CAN_h_
#include "sys.h"

#define abs(x) ((x)>0? (x):(-(x)))

typedef struct
{
	
	union
	{
		u8 DATA_8[2];
		u16 DATA_16;
	}Left;
	
	union
	{
		u8 DATA_8[2];
		u16 DATA_16;
	}Back;
	
	
}PS_TYPE;													//光电传感器变量


void CAN1_Configuration(void);


#endif




