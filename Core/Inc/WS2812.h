#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "stm32g4xx_hal.h"

#define WS2812_NUM_LEDS 2

#define WS2812_HI_VAL 28
#define WS2812_LO_VAL 10

#define WS2812_BUF_LEN ((WS2812_NUM_LEDS*24)+1)

#define WS2812_POS_1 0
#define WS2812_POS_2 1
#define WS2812_POS_ALL 2

typedef struct {
	TIM_HandleTypeDef *tim;
	uint32_t tim_channel;
} WS2812;

typedef struct {
  uint8_t pos;
  uint8_t r;
  uint8_t g;
  uint8_t b;
} WS2812_LED;

void WS2812_Init(WS2812 *dev, TIM_HandleTypeDef *timer, uint32_t channel);
void WS2812_ResetBuf(uint8_t *buf);
void WS2812_WriteBuf(uint8_t *buf, uint8_t r, uint8_t g, uint8_t b, uint8_t led);
HAL_StatusTypeDef WS2812_Send(WS2812 *dev, uint8_t *buf);
HAL_StatusTypeDef WS2812_StopDMA(WS2812 *dev);

#endif /* INC_WS2812_H_ */
