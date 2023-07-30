#ifndef __SPI_DRV_H
#define __SPI_DRV_H

void spi_init(void);
uint8_t spi_dev_init(uint8_t dev);
uint8_t spi_dev_cs(uint8_t dev, uint8_t line, uint8_t state);
uint8_t spi_dev_transfer(uint8_t dev, uint8_t *send, uint8_t *recv, uint16_t size);
uint8_t spi_dev_deinit(uint8_t dev);

#endif

