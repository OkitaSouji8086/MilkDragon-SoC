#ifndef LED_H
#define LED_H

#include "common_func.h"

#define LEDS_BASEADDR 0xbf20f300

#define LEDS_GPIO_DATA (LEDS_BASEADDR + 0x00)

// set leds pin
void setLedPin(uint32_t data);
void toggleLedPin(uint32_t data);

#endif 
