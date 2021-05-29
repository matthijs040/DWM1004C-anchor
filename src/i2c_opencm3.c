#include "../inc/i2c_opencm3.h"

#ifndef STM32L0
#define STM32L0
#endif

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <string.h>
#include <stdio.h>

#define sizeof_array(array)		( sizeof(array) / sizeof(array[0]) )

static void i2c_read(uint8_t i2c_addr, uint8_t initial_register, uint8_t* data, size_t registers_to_read)
{  
    i2c_transfer7(I2C1, i2c_addr, &initial_register , sizeof(initial_register) , data, registers_to_read );
	
}

static void i2c_write(uint8_t i2c_addr, uint8_t initial_register, uint8_t* data, size_t registers_to_write)
{
	uint8_t bytes_to_write[registers_to_write + sizeof(initial_register)];
	bytes_to_write[0] = initial_register;
	memcpy(bytes_to_write + sizeof(initial_register), data, registers_to_write);	

    i2c_transfer7(I2C1, i2c_addr, bytes_to_write , sizeof_array(bytes_to_write) , NULL, 0 );
}

#define I2C_PINS	( GPIO9 | GPIO10)
#define I2C_PORT	GPIOA
#define I2C_AF_ID	GPIO_AF1

i2c_link_t i2c_link_init(void)
{
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_set_peripheral_clk_sel(I2C1, RCC_CCIPR_I2C1SEL_HSI16);

	// Accelerate the MSI clock (the only one that is active in the uC's reset state.)
    i2c_reset(I2C1);

	// Setup GPIO pin 6 and 7 on GPIO port B for alternate function. 
	gpio_mode_setup(I2C_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C_PINS);
    // Set alternate function to i2c.
    // In page 55 of datasheet. I2C1-SCL and SDA.
	gpio_set_af(I2C_PORT, I2C_AF_ID, I2C_PINS);
	i2c_peripheral_disable(I2C1);
	//configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, 0);

	uint32_t clk = rcc_get_i2c_clk_freq(I2C1);
	
	printf("clk: %ld", clk );

	i2c_set_speed(I2C1, i2c_speed_fm_400k, (clk / 1000000) );
	//configure No-Stretch CR1 (only relevant in slave mode)
	i2c_enable_stretching(I2C1);
	//addressing mode
	i2c_set_7bit_addr_mode(I2C1);
	

	i2c_peripheral_enable(I2C1);
    
    i2c_link_t ret = { i2c_link_init, i2c_read, i2c_write};   
    return ret; 
}

