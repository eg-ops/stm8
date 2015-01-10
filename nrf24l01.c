#include "nrf24l01.h"



uint8_t nrf24l01p_spi_rw(uint8_t value) {
	while (SPI_GetFlagStatus(SPI1, SPI_FLAG_TXE) == RESET) {
	}
	SPI_SendData(SPI1, value);
	while (SPI_GetFlagStatus(SPI1, SPI_FLAG_RXNE) == RESET) {
	}
	return SPI_ReceiveData(SPI1);
}

uint8_t nrf24l01p_read(uint8_t reg, uint8_t * mem, int size) {
        nrf24l01p_irq_disable();
        nrf24l01p_csn_low();
	uint8_t status = nrf24l01p_spi_rw(reg);
	while (size--) {
		*mem++ = nrf24l01p_spi_rw(0);
	}
        nrf24l01p_csn_high();
        nrf24l01p_irq_enable();
	return status;
}

uint8_t nrf24l01p_write(uint8_t reg, uint8_t * mem, int size) {
        nrf24l01p_irq_disable();
        nrf24l01p_csn_low();
	uint8_t status = nrf24l01p_spi_rw(reg);
	while (size--) {
		nrf24l01p_spi_rw(*mem++);
	}
        nrf24l01p_csn_high();
        nrf24l01p_irq_enable();
	return status;
}


uint8_t nrf24l01p_read_byte(uint8_t reg){
    uint8_t buff;
    nrf24l01p_read(reg, &buff, sizeof(buff));
    return buff;
}

void nrf24l01p_write_byte(uint8_t reg, uint8_t value){
    uint8_t buff = value;
    nrf24l01p_write(reg, &buff, sizeof(buff));
}