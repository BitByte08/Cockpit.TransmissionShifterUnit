#ifndef __GEAR_CAN_H
#define __GEAR_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

HAL_StatusTypeDef GearCan_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef GearCan_Send(CAN_HandleTypeDef *hcan, uint8_t gearValue);

#ifdef __cplusplus
}
#endif

#endif /* __GEAR_CAN_H */
