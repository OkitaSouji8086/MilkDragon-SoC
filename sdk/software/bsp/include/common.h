#pragma once

#define __IO volatile
#define __I volatile const
#define __O volatile

// physical addr
#define ISRAM_BASE (0x1C000000)
#define ESRAM_BASE (0x1C400000)
// #define UART_BASE (0x1F000000)
// #define X2P_BASE (0x1F100000)
// #define DMAC_BASE (X2P_BASE + 0x10000)
// #define CONFREG_BASE (0x1F200000)

// virtual addr
// #define ISRAM_BASE (0xBC000000)
// #define ESRAM_BASE (0xBC400000)
#define X2P_BASE (0xBF100000)
#define DMAC_BASE (X2P_BASE + 0x10000)
#define CONFREG_BASE (0xBF200000)