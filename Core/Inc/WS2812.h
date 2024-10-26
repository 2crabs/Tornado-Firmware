#ifndef INC_WS2812_H_
#define INC_WS2812_H_

#include "stm32g4xx_hal.h"

#define WS2812_NUM_LEDS 2

#define WS2812_HI_VAL 28
#define WS2812_LO_VAL 10

#define WS2812_BUF_LEN ((WS2812_NUM_LEDS*24)+1)

#define RGB_TYPE_SETTING_ID 0b10000
#define RGB_TYPE_ERR_TERMINATION 0b1000
#define RGB_TYPE_ERR_PERIODIC 0b100
#define RGB_TYPE_CUSTOM 0b10
#define RGB_TYPE_ID 0b1

typedef struct {
	TIM_HandleTypeDef *tim;
	uint32_t tim_channel;
} WS2812;

//only set data to color if current type
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t type;
  uint8_t enabled;
} RGBState;

void WS2812_Init(WS2812 *dev, TIM_HandleTypeDef *timer, uint32_t channel);
void WS2812_ResetBuf(uint8_t *buf);
void WS2812_WriteBuf(uint8_t *buf, uint8_t r, uint8_t g, uint8_t b, uint8_t led);
HAL_StatusTypeDef WS2812_Send(WS2812 *dev, uint8_t *buf);
HAL_StatusTypeDef WS2812_StopDMA(WS2812 *dev);

#endif /* INC_WS2812_H_ */
