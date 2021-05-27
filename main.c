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

#define DWM_CS_PORT    GPIOB
#define DWM_CS_PIN     GPIO0

#define LIS3DH_DEV_ADDR 0x33 // 0b00110011
#define LIS3DH_WAI_REG  0x0F
#define LIS3_CS_PIN GPIO15
#define LIS3_CS_PORT   GPIOA


static void clock_setup(void)
{
	rcc_osc_on(RCC_HSI16);
    rcc_wait_for_osc_ready(RCC_HSI16);
}


int main(void)
{
    // Setting the "LIS3 PWR" pin in attempt to enable the acc.
    clock_setup();
    
    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(LIS3_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,LIS3_CS_PIN );
    
    i2c_link_t i2c = i2c_link_init();
    
    uint8_t wai_data = 0;
    i2c.read(LIS3DH_DEV_ADDR, LIS3DH_WAI_REG, &wai_data, 1);

    printf("wai_data: %d\n", wai_data);


    // GPIO port B is the port on which the CS pin is.
    rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup( DWM_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, DWM_CS_PIN );
    gpio_set(DWM_CS_PORT, DWM_CS_PIN);
    
    spi_link_t spi = spi_link_init();
    
    uint8_t request = 0;

    gpio_clear(DWM_CS_PORT, DWM_CS_PIN);
    spi.write( &request, 1 );

    uint8_t response[4];
    spi.read( response, sizeof_arr(response) );
    gpio_set(DWM_CS_PORT, DWM_CS_PIN);

    while(true);
}