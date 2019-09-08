#ifndef __AJSR_H
#define __AJSR_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
void Initial_UART3(u32 bound);
void SRInit(void) ;
float SRGetDistance(void);
void SRStartRanging(void);	 				    
#endif
