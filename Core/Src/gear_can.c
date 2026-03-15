#include "gear_can.h"

#define GEAR_CAN_STD_ID 0x301U
#define GEAR_CAN_DLC    1U

static HAL_StatusTypeDef GearCan_FilterConfig_AllPass(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.FilterBank = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  return HAL_CAN_ConfigFilter(hcan, &filter);
}

HAL_StatusTypeDef GearCan_Init(CAN_HandleTypeDef *hcan)
{
  if (GearCan_FilterConfig_AllPass(hcan) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_CAN_Start(hcan);
}

HAL_StatusTypeDef GearCan_Send(CAN_HandleTypeDef *hcan, uint8_t gearValue)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[GEAR_CAN_DLC] = {0};
  uint32_t txMailbox = 0U;

  txHeader.StdId = GEAR_CAN_STD_ID;
  txHeader.ExtId = 0U;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = GEAR_CAN_DLC;
  txHeader.TransmitGlobalTime = DISABLE;

  txData[0] = gearValue;

  return HAL_CAN_AddTxMessage(hcan, &txHeader, txData, &txMailbox);
}
