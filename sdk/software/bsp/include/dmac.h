#pragma once

#include "common_func.h"
#include "common.h"

typedef struct
{
    __IO U32 CMD_REG0;         // 0x00
    __IO U32 CMD_REG1;         // 0x04
    __IO U32 CMD_REG2;         // 0x08
    __IO U32 CMD_REG3;         // 0x0C
    __IO U32 STATIC_REG0;      // 0x10
    __IO U32 STATIC_REG1;      // 0x14
    __IO U32 STATIC_REG2;      // 0x18
    __IO U32 STATIC_REG3;      // 0x1C
    __IO U32 STATIC_REG4;      // 0x20
    U32 RESERVED0[2];          // 0x24 - 0x28
    __I U32 RESTRICT_REG;      // 0x2C
    __I U32 READ_OFFSET_REG;   // 0x30
    __I U32 WRITE_OFFSET_REG;  // 0x34
    __I U32 FIFO_FULLNESS_REG; // 0x38
    __I U32 CMD_OUTS_REG;      // 0x3C
    __IO U32 CH_ENABLE_REG;    // 0x40
    __O U32 CH_START_REG;      // 0x44
    __I U32 CH_ACTIVE_REG;     // 0x48
    U32 RESERVED1[1];          // 0x4C
    __I U32 COUNT_REG;         // 0x50
    U32 RESERVED2[19];         // 0x54 - 0x9C
    __IO U32 INT_RAWSTAT_REG;  // 0xA0
    __O U32 INT_CLEAR_REG;     // 0xA4
    __IO U32 INT_ENABLE_REG;   // 0xA8
    __O U32 INT_STATUS_REG;    // 0xAC
} DMAC_TypeDef;

#define DMA1 ((DMAC_TypeDef *)DMAC_BASE)