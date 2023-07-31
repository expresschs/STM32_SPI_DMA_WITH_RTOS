#include "stm32h7xx_hal.h"
#include "spi_def.h"
#include "spi_cfg.h"

/* SPI1 define */
#define SPI1_LINE_PA15              (0U)
#define SPI1_LINE_PB3               (1U)
#define SPI1_CACHE_SIZE             (4 * 1024UL)

extern SPI_HandleTypeDef hspi1;
static uint8_t spi1_tx_buf[SPI1_CACHE_SIZE] = {0};
static uint8_t spi1_rx_buf[SPI1_CACHE_SIZE] = {0};

static inline void _spi1_select(uint8_t line, uint8_t cs)
{
    if (SPI1_LINE_PA15 == line) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, (GPIO_PinState)cs);
    } else if (SPI1_LINE_PB3 == line) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (GPIO_PinState)cs);
    }
}

static inline void _spi1_setup(uint8_t line)
{
    if (SPI1_LINE_PA15 == line) {
        /* TODO: Setup Params, Baudrate Phase Polarity ...ETC... */
    } else if (SPI1_LINE_PB3 == line) {
        /* Nothing To Do */
    }
}


/* SPI2 define */
/* TODO */


static spi_ctx g_cfg_tbl[] = {
    {&hspi1, _spi1_select, _spi1_setup, spi1_tx_buf, spi1_rx_buf, 0, SPI1_CACHE_SIZE, NULL, NULL},
    //{&hspi2, _spi2_select, _spi2_setup, spi2_tx_buf, spi2_rx_buf, 0, SPI2_CACHE_SIZE, NULL, NULL},
}; 

void spi_cfg_get(void **cfg, uint16_t *size)
{
    *cfg = (void *)g_cfg_tbl;
    *size = sizeof(g_cfg_tbl)/sizeof(g_cfg_tbl[0]);

    return;
}

