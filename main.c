#ifndef STM32L0 // For intellisense.
#define STM32L0
#endif

#include "inc/spi_opencm3.h"
#include "inc/i2c_opencm3.h"

#include <stdbool.h>
#include <stdio.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define sizeof_arr(arr) ( sizeof(arr) / sizeof(arr[0]) )

#define GPIO_CS_PORT    GPIOB
#define GPIO_CS_PIN     GPIO0

#define LIS3DH_DEV_ADDR 0x33
#define LIS3DH_WAI_REG  0x0F

int main(void)
{

    i2c_link_t i2c = i2c_link_init();
    
    uint8_t wai_data = 0;
    i2c.read(LIS3DH_DEV_ADDR, LIS3DH_WAI_REG, &wai_data, 1);

    printf("wai_data: %d\n", wai_data);









    // GPIO port B is the port on which the CS pin is.
    rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup( GPIO_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_CS_PIN );
    gpio_set(GPIO_CS_PORT, GPIO_CS_PIN);
    
    spi_link_t spi = spi_link_init();
    
    uint8_t request = 0;

    gpio_clear(GPIO_CS_PORT, GPIO_CS_PIN);
    spi.write( &request, 1 );

    uint8_t response[4];
    spi.read( response, sizeof_arr(response) );
    gpio_set(GPIO_CS_PORT, GPIO_CS_PIN);

    while(true);
}