#ifndef STM32L0 // For intellisense.
#define STM32L0
#endif

#include "inc/spi_opencm3.h"
#include <stdbool.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define sizeof_arr(arr) ( sizeof(arr) / sizeof(arr[0]) )

#define GPIO_CS_PORT    GPIOB
#define GPIO_CS_PIN     GPIO1


int main(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup( GPIO_CS_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_CS_PIN );
    gpio_set(GPIO_CS_PORT, GPIO_CS_PIN);
    // enable pullup as the default state so that no SPI transaction is initiated by floating voltage issues?
    
    spi_link_t link = spi_link_init();
    
    uint8_t request = 0;

    gpio_clear(GPIO_CS_PORT, GPIO_CS_PIN);
    link.write( &request, 1 );

    uint8_t response[4];
    link.read( response, sizeof_arr(response) );
    gpio_set(GPIO_CS_PORT, GPIO_CS_PIN);

    while(true);
}