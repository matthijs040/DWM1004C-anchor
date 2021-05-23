#ifndef STM32L0 // For intellisense.
#define STM32L0
#endif

#include "inc/spi_opencm3.h"
#include <stdbool.h>

#define sizeof_arr(arr) sizeof(arr) / sizeof(arr[0])

int main(void)
{
    spi_link_t link = spi_link_init();
    
    uint8_t request = 0;
    link.write( &request, 1 );

    uint8_t response[4];
    link.read( response, sizeof_arr(response) );

    while(true);
}