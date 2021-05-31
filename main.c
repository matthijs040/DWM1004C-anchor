#ifndef STM32L0 // For intellisense.
#define STM32L0
#endif

#include "inc/spi_opencm3.h"
#include "inc/i2c_opencm3.h"
#include "inc/mpu6050.h"

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define sizeof_arr(arr) ( sizeof(arr) / sizeof(arr[0]) )

#define DWM_CS_PORT    GPIOB
#define DWM_CS_PIN     GPIO0

#define LIS3DH_DEV_ADDR 0x33 // 0b00110011
#define LIS3DH_WAI_REG  0x25
#define LIS3_CS_PIN     GPIO15
#define LIS3_PWR_PIN    GPIO4
#define LIS3_CS_PORT    GPIOA

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
    
    i2c_link_t i2c_link = i2c_link_init();
    mpu_t mpu_sensor = mpu_init(i2c_link, false);

    if( mpu_read_wai_register(mpu_sensor) )
	{
        		puts("Initial read request returned correctly.");
    }

    /**
    rcc_periph_clock_enable(RCC_GPIOA);
	gpio_mode_setup(LIS3_CS_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, LIS3_CS_PIN | LIS3_PWR_PIN );

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
    **/

    while(true);
}