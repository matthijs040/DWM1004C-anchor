#include "../inc/spi_opencm3.h"

#ifndef STM32L0 // for intellisense.
#define STM32L0
#endif

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <stdio.h>

bool extended_data_frame()
{
    // The registers are already dereferenced.
    // The Control registers state is checked here if the DFF bit is set (implying a 16bit frame size)
    // return SPI1_CR1 && SPI_CR1_DFF_16BIT;
    return false;
}

/**
 * @brief Doing a run of sequential reads of the SPI data register specified in the init sequence.
 * If values are read in 16 bit strides the most significant of the two bytes comes first. 
 * 
 * @param data 
 * @param count 
 */
void read(uint8_t* data, const size_t count )
{
    if(extended_data_frame())
    {
        for(size_t i = 0; i < count; i+=2) 
        {
            uint16_t ret = spi_read(SPI1);
            data[i] = ret >> sizeof(uint8_t); 
            data[i + 1] = ret;
        }
        
    }
    else
    {
        for(size_t i  = 0; i < count; i++) 
        {
            data[i] = (uint8_t)spi_read(SPI1);
            
        }
    }    
}
	
void write(uint8_t* data, const size_t count)
{
    if( extended_data_frame() )
        for(size_t i  = 0; i < count; i+=2) 
            spi_write(SPI1, (uint16_t)(data[i] | data[i + 1]) );

    else
        for(size_t i  = 0; i < count; i++) 
            spi_write(SPI1, data[i]);  
}

#define GPIO_SPI_PINS ( GPIO5 | GPIO6 | GPIO7 )

spi_link_t spi_link_init()
{  

    spi_reset(SPI1);
    spi_disable(SPI1);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_GPIOA);    
    
	gpio_mode_setup( GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_SPI_PINS );
    // Set alternate function on SPI pins to 0 (the SPI alternate function). 
    // Written on page 44 of the stm32-l0 datasheet.
	gpio_set_af(GPIOA, GPIO_AF0, GPIO_SPI_PINS);

    spi_init_master( SPI1
                   , SPI_CR1_BAUDRATE_FPCLK_DIV_2
                   , SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE
                   , SPI_CR1_CPHA_CLK_TRANSITION_2
                   , SPI_CR1_DFF_8BIT
                   , SPI_CR1_MSBFIRST );

    spi_set_full_duplex_mode(SPI1);
    spi_set_nss_low(SPI1);
    spi_set_frf_motorola(SPI1);

    spi_enable(SPI1);

    uint32_t spi_speed = rcc_get_spi_clk_freq(SPI1);
    printf("spi speed: %ld\n", spi_speed);


    spi_link_t ret = { spi_link_init, read, write };
    return ret;
}