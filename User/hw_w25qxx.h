#ifndef _HW_W25QXX_H_
#define _HW_W25QXX_H_

//erase 4k align (4096 = 256 * 16)
#define W25QXX_ERASE_SHIFT              12
#define W25QXX_ERASE_SIZE               (0x01 << W25QXX_ERASE_SHIFT)
#define W25QXX_ERASE_ADDR_MASK          ((~0U) << W25QXX_ERASE_SHIFT)
//block erase 64k aligned
#define W25QXX_BLOCK_ERASE_SHIFT        16
#define W25QXX_BLOCK_ERASE_SIZE         (0x01 << W25QXX_BLOCK_ERASE_SHIFT)
#define W25QXX_BLOCK_ERASE_ADDR_MASK    ((~0U) << W25QXX_BLOCK_ERASE_SHIFT)

//write address 256 aligned
#define W25QXX_WRITE_SHIFT              8
#define W25QXX_WRITE_SIZE               (0x01 << W25QXX_WRITE_SHIFT)
#define W25QXX_WRITE_ADDR_MASK          ((~0U) << W25QXX_WRITE_SHIFT)

uint8_t w25qxx_read(uint32_t addr, uint8_t *buffer, uint32_t size);
uint8_t w25qxx_write(uint32_t addr, uint8_t *buffer, uint32_t size);
uint8_t w25qxx_erase(uint32_t addr, uint32_t size);
uint8_t w25qxx_erase_page(uint32_t page);
uint8_t w25qxx_get_id(uint16_t *id);

#endif

