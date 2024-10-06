/*
 * M34E04.h
 *
 *  Created on: Oct 5, 2024
 */

#ifndef INC_M34E04_H_
#define INC_M34E04_H_

#include "stm32g4xx_hal.h"

#define M34E04_ADDR (0b1010000 << 1)
#define M34E04_SET_PAGE0 (0b0110110 << 1)
#define M34E04_SET_PAGE1 (0b0110111 << 1)
#define M34E04_READ_PAGE (0b0110110 << 1)

HAL_StatusTypeDef M34E04_SetPageToZero(I2C_HandleTypeDef* handle);
HAL_StatusTypeDef M34E04_SetPageToOne(I2C_HandleTypeDef* handle);
HAL_StatusTypeDef M34E04_StartRead(I2C_HandleTypeDef* handle, uint8_t addr, uint8_t* buff);
HAL_StatusTypeDef M34E04_Write(I2C_HandleTypeDef* handle, uint8_t addr, uint8_t* data, uint8_t size);



#endif /* INC_M34E04_H_ */
