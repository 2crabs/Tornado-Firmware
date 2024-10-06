#include "M34E04.h"

HAL_StatusTypeDef M34E04_SetPageToZero(I2C_HandleTypeDef* handle){
  uint8_t dummyData;
  return HAL_I2C_Master_Transmit_IT(handle, M34E04_SET_PAGE0, &dummyData, 0);
}

HAL_StatusTypeDef M34E04_SetPageToOne(I2C_HandleTypeDef* handle){
  uint8_t dummyData;
  return HAL_I2C_Master_Transmit_IT(handle, M34E04_SET_PAGE1, &dummyData, 0);
}

HAL_StatusTypeDef M34E04_StartRead(I2C_HandleTypeDef* handle, uint8_t addr, uint8_t* buff){
  return HAL_I2C_Mem_Read_IT(handle, M34E04_ADDR, addr, I2C_MEMADD_SIZE_8BIT, buff, 1);
}

HAL_StatusTypeDef M34E04_Write(I2C_HandleTypeDef* handle, uint8_t addr, uint8_t* data, uint8_t size){
  return HAL_I2C_Mem_Write_IT(handle, M34E04_ADDR, addr, I2C_MEMADD_SIZE_8BIT, data, size);
}
