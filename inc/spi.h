#ifndef SPI_H
#define SPI_H

#include <stdint.h>	// uint8_t
#include <stdlib.h> // size_t

typedef struct spi_link_t {
    struct spi_link_t (*init)(void);

	void (*read)(uint8_t    spi_addr
				,uint8_t*   data
				,size_t     count);
	
	void (*write)(uint8_t   spi_addr
				, uint8_t*  data
				, size_t    count);
}spi_link_t;

#endif 