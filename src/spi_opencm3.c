#include "../inc/spi_opencm3.h"

#ifndef STM32L0 // for intellisense.
#define STM32L0
#endif

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define GPIO_SPI_PINS ( GPIO5 | GPIO6 | GPIO7 )

void read(uint8_t spi_addr,uint8_t* data, size_t count )
{

}
	
void write(uint8_t spi_addr, uint8_t* data, size_t count)
{
}

spi_link_t init()
{  
    spi_disable(SPI1);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_GPIOA);
    
	gpio_mode_setup( GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_SPI_PINS );
    // Set alternate function on SPI pins to 0 (the SPI alternate function). 
    // Written on page 44 of the stm32-l0 datasheet.
	gpio_set_af(GPIOA, GPIO_AF0, GPIO_SPI_PINS);

    spi_init_master( SPI1
                   , SPI_CR1_BAUDRATE_FPCLK_DIV_16
                   , SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE
                   , SPI_CR1_CPHA_CLK_TRANSITION_1
                   , SPI_CR1_DFF_8BIT
                   , SPI_CR1_MSBFIRST );

    spi_enable(SPI1);
    


    spi_link_t ret = { init, read, write };
    return ret;
}