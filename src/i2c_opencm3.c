#include "../inc/i2c_opencm3.h"

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <string.h>
#include <stdio.h>

#define sizeof_array(array)		( sizeof(array) / sizeof(array[0]) )

bool i2c_timeout_reached(uint32_t* timeout)
{
	if(timeout)
		return (*timeout)-- == 0;
	
	return false;
}

/**
 * Run a write/read transaction to a given 7bit i2c address
 * If both write & read are provided, the read will use repeated start.
 * Both write and read are optional
 * @param i2c peripheral of choice, eg I2C1
 * @param addr 7 bit i2c device address
 * @param w buffer of data to write
 * @param wn length of w
 * @param r destination buffer to read into
 * @param rn number of bytes to read (r should be at least this long)
 */
void i2c_transfer7b(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn, uint32_t* timeout)
{
	/*  waiting for busy is unnecessary. read the RM */
	if (wn) {
		i2c_set_7bit_address(i2c, addr);
		i2c_set_write_transfer_dir(i2c);
		i2c_set_bytes_to_transfer(i2c, wn);
		if (rn) {
			i2c_disable_autoend(i2c);
		} else {
			i2c_enable_autoend(i2c);
		}
		i2c_send_start(i2c);

		while (wn--) {
			bool wait = true;
			while (wait) {
				if (i2c_transmit_int_status(i2c)) {
					wait = false;
				}
				while (i2c_nack(i2c))  /* FIXME Some error */
					if(i2c_timeout_reached(timeout))
						return;
			}
			i2c_send_data(i2c, *w++);
		}
		/* not entirely sure this is really necessary.
		 * RM implies it will stall until it can write out the later bits
		 */
		if (rn) {
			while (!i2c_transfer_complete(i2c))
				if(i2c_timeout_reached(timeout))
					return;
		}
	}

	if (rn) {
		/* Setting transfer properties */
		i2c_set_7bit_address(i2c, addr);
		i2c_set_read_transfer_dir(i2c);
		i2c_set_bytes_to_transfer(i2c, rn);
		/* start transfer */
		i2c_send_start(i2c);
		/* important to do it afterwards to do a proper repeated start! */
		i2c_enable_autoend(i2c);

		for (size_t i = 0; i < rn; i++) {
			while (i2c_received_data(i2c) == 0)
				if(i2c_timeout_reached(timeout))
					return;
			
			r[i] = i2c_get_data(i2c);
		}
	}
}

static void i2c_read(uint8_t i2c_addr, uint8_t initial_register, uint8_t* data, size_t registers_to_read)
{  
	
	uint32_t timeout = rcc_get_i2c_clk_freq(I2C1) / 10000;
    i2c_transfer7b(I2C1, i2c_addr, &initial_register , sizeof(initial_register) , data, registers_to_read, &timeout );
	
}

static void i2c_write(uint8_t i2c_addr, uint8_t initial_register, uint8_t* data, size_t registers_to_write)
{
	uint32_t timeout = rcc_get_i2c_clk_freq(I2C1) / 10000;
	uint8_t bytes_to_write[registers_to_write + sizeof(initial_register)];
	bytes_to_write[0] = initial_register;
	memcpy(bytes_to_write + sizeof(initial_register), data, registers_to_write);	

    i2c_transfer7b(I2C1, i2c_addr, bytes_to_write , sizeof_array(bytes_to_write) , NULL, 0, &timeout);
}

#define I2C1_PINS	( GPIO9 | GPIO10)
#define I2C1_PORT	GPIOA
#define I2C1_AF_ID	GPIO_AF1

i2c_link_t i2c_link_init(void)
{
	// Enable the 16MHz internal oscillator if it is not enabled.
	// Then wait for it to be ready and assign it as the I2C clock.
	if( !rcc_is_osc_ready(RCC_HSI16) )
		rcc_osc_on(RCC_HSI16);
	
	rcc_wait_for_osc_ready(RCC_HSI16);
	
	rcc_set_peripheral_clk_sel(I2C1, RCC_CCIPR_I2C1SEL_HSI16);

	// Then enable the periphrial clocks that will drive I2c.
	rcc_periph_clock_enable(RCC_I2C1);
	rcc_periph_clock_enable(RCC_GPIOA);

	// Setup GPIO pin 6 and 7 on GPIO port B for alternate function. 
	gpio_mode_setup(I2C1_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, I2C1_PINS);
    // Set alternate function to i2c.
    // In page 55 of datasheet. I2C1-SCL and SDA.
	gpio_set_af(I2C1_PORT, I2C1_AF_ID, I2C1_PINS);

	i2c_peripheral_disable(I2C1);
	i2c_reset(I2C1);

	//configure ANFOFF DNF[3:0] in CR1
	i2c_enable_analog_filter(I2C1);
	i2c_set_digital_filter(I2C1, 0);
	
	// This is here for debugging
	uint32_t clk = rcc_get_i2c_clk_freq(I2C1);
	printf("clk: %ld", clk );

	i2c_set_speed(I2C1, i2c_speed_fm_400k, ( clk / 1000000 ) * 4 );
	

	i2c_enable_stretching(I2C1);
	i2c_set_7bit_addr_mode(I2C1);

	i2c_peripheral_enable(I2C1);
    
    i2c_link_t ret = { i2c_link_init, i2c_read, i2c_write};   
    return ret; 
}

