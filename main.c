#ifndef STM32L0 // For intellisense.
#define STM32L0
#endif

#include "inc/spi_opencm3.h"
#include "inc/i2c_opencm3.h"
#include "inc/mpu6050.h"
#include "inc/lis3dh.h"

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define sizeof_arr(arr) ( sizeof(arr) / sizeof(arr[0]) )

#define DWM_CS_PORT    GPIOB
#define DWM_CS_PIN     GPIO0

#define LIS3_CS_PIN     GPIO15
#define LIS3_PWR_PIN    GPIO4
#define LIS3_CS_PORT    GPIOA

#define delay(clkcycls) for (int i = 0; i < clkcycls; i++) __asm__("nop")

/* 16MHz * pll_mult / pll_div / hpre / ppreX */
const struct rcc_clock_scale rcc_hsi_configs[] = {
	{ /* 24 MHz -> 16 * 3 / 2 = 24 */
    	.pll_source       = RCC_CFGR_PLLSRC_HSI16_CLK,
        .pll_mul          = RCC_CFGR_PLLMUL_MUL3,
        .pll_div          = RCC_CFGR_PLLDIV_DIV2,

        .flash_waitstates = 1,

        .voltage_scale    = PWR_SCALE1,

		.hpre   = RCC_CFGR_HPRE_NODIV,
		.ppre1  = RCC_CFGR_PPRE1_NODIV,  // 24 MHz for ppre1
		.ppre2  = RCC_CFGR_PPRE2_NODIV,  // 24

		.ahb_frequency	= 24000000,
		.apb1_frequency = 24000000,
		.apb2_frequency = 24000000,
        .msi_range  = RCC_ICSCR_MSIRANGE_2MHZ
	},
    { /* 12 MHz -> 16 * 3 / 4 = 12 */
    	.pll_source       = RCC_CFGR_PLLSRC_HSI16_CLK,
		.pll_mul          = RCC_CFGR_PLLMUL_MUL3,
        .pll_div          = RCC_CFGR_PLLDIV_DIV4,

        .flash_waitstates = 0,

        .voltage_scale    = PWR_SCALE1,

		.hpre   = RCC_CFGR_HPRE_NODIV,
		.ppre1  = RCC_CFGR_PPRE1_NODIV,  // 12 MHz for ppre1
		.ppre2  = RCC_CFGR_PPRE2_NODIV,  // 12

		.ahb_frequency	= 12000000,
		.apb1_frequency = 12000000,
		.apb2_frequency = 12000000,
        .msi_range  = RCC_ICSCR_MSIRANGE_2MHZ
	}
};

int _write(int file, char *ptr, int len)
{
    (void) file;
    (void) ptr;
    (void) len;
    errno = EIO;
    return -1;
}


static void clock_setup(void)
{  
	rcc_clock_setup_pll(&rcc_hsi_configs[0]);
}


int main(void)
{
    // Setting the "LIS3 PWR" pin in attempt to enable the acc.
    clock_setup();
      
    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(LIS3_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LIS3_CS_PIN | LIS3_PWR_PIN );

    i2c_link_t i2c = i2c_link_init();
        
    gpio_clear(LIS3_CS_PORT, LIS3_PWR_PIN);
    delay(100000);
    gpio_set(LIS3_CS_PORT, LIS3_PWR_PIN);
    delay(1000);

    lis3dh_t lis3_sensor = lis3dh_init(i2c, LIS3DH_I2C_ADDRESS_2);

    while( !lis3dh_read_whoami(lis3_sensor))
    {
        delay(1000);
    }
    
    /**
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
    **/

    return EXIT_SUCCESS;
}