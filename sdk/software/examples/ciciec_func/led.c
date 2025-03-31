#include "led.h"

// set leds pin
void setLedPin(uint32_t data)
{
    RegWrite(LEDS_GPIO_DATA,data);
}

// toggle one led pin
void toggleLedPin(uint32_t data)
{
    uint32_t led_data;
    uint32_t process_bit;

    led_data = RegRead(LEDS_GPIO_DATA);
    process_bit = 0x1 << data;

    if(led_data & process_bit)
    {
        led_data = led_data & (~process_bit);
    }
    else
    {
        led_data = led_data | process_bit;
    }

    RegWrite(LEDS_GPIO_DATA,led_data);
}