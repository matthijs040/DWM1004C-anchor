#ifndef STM32L0 // For intellisense.
#define STM32L0
#endif

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>

int main(void)
{
    volatile int number = 5;
    number++;
}