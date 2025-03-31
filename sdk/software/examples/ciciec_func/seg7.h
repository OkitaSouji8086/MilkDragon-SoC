// seg7.h

#ifndef SEG7_H
#define SEG7_H

#include "common_func.h"

#define SEG7_BASEADDR 0xbf20f200

#define SEG7_SELECT (SEG7_BASEADDR + 0x00)

#define SEG7_NUM (SEG7_BASEADDR + 0x04)

// set seg num
void setSegNum(uint32_t seg1,uint32_t num1,uint32_t seg2,uint32_t num2);

#endif
