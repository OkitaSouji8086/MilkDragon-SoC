// seg7.c

#include "seg7.h"

void setSegNum(uint32_t seg1,uint32_t num1,uint32_t seg2,uint32_t num2)
{
    // select seg
    uint32_t select_seg = ((seg1<<2) + seg2);

    RegWrite(SEG7_SELECT,select_seg);

    // set num
    uint32_t num_seg = ((num1<<4) + num2);

    RegWrite(SEG7_NUM,num_seg);

}