/*
 * dw1000_stm32.h
 *
 *  Created on: 11 Feb 2020
 *      Author: refa
 */

#ifndef DW1000_STM32_H_
#define DW1000_STM32_H_

#include "dw1000.h"

void dw1000_WriteData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t header[] = {(1 << 7) | reg};
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dw1000->spi, header, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(dw1000->spi, data, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

void dw1000_ReadData(dw1000_HandleTypeDef *dw1000, uint8_t reg, uint8_t *data, uint8_t len) {
  uint8_t header[] = {(0 << 7) | reg};
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(dw1000->spi, header, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(dw1000->spi, data, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

void dw1000_SSBlink(dw1000_HandleTypeDef *dw1000) {
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_RESET);
  HAL_Delay(2000);
  HAL_GPIO_WritePin(dw1000->ss_port, dw1000->ss_pin, GPIO_PIN_SET);
}

#endif /* DW1000_STM32_H_ */
