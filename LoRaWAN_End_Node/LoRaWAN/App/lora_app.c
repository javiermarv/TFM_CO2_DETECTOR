/**
  ******************************************************************************
  * @file    lora_app.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "Region.h" /* Needed for LORAWAN_DEFAULT_DATA_RATE */
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "lora_app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "stm32_lpm.h"
#include "adc_if.h"
#include "sys_conf.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
  * @brief LoRa State Machine states
  */
typedef enum TxEventType_e
{
  /**
    * @brief AppdataTransmition issue based on timer every TxDutyCycleTime
    */
  TX_ON_TIMER,
  /**
    * @brief AppdataTransmition external event plugged on OnSendEvent( )
    */
  TX_ON_EVENT
  /* USER CODE BEGIN TxEventType_t */

  /* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  LoRa endNode send request
  * @param  none
  * @retval none
  */
static void SendTxData(void);

/**
  * @brief  TX timer callback function
  * @param  timer context
  * @retval none
  */
static void OnTxTimerEvent(void *context);

/**
  * @brief  LED Tx timer callback function
  * @param  LED context
  * @retval none
  */
static void OnTxTimerLedEvent(void *context);

/**
  * @brief  LED Rx timer callback function
  * @param  LED context
  * @retval none
  */
static void OnRxTimerLedEvent(void *context);

/**
  * @brief  LED Join timer callback function
  * @param  LED context
  * @retval none
  */
static void OnJoinTimerLedEvent(void *context);

/**
  * @brief  join event callback function
  * @param  params
  * @retval none
  */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
  * @brief  tx event callback function
  * @param  params
  * @retval none
  */
static void OnTxData(LmHandlerTxParams_t *params);

/**
  * @brief callback when LoRa endNode has received a frame
  * @param appData
  * @param params
  * @retval None
  */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/*!
 * Will be called each time a Radio IRQ is handled by the MAC layer
 *
 */
static void OnMacProcessNotify(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
  * @brief User application buffer
  */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
  * @brief User application data structure
  */
static LmHandlerAppData_t AppData = { 0, 0, AppDataBuffer };

static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
  * @brief LoRaWAN handler Callbacks
  */
static LmHandlerCallbacks_t LmHandlerCallbacks =
{
  .GetBatteryLevel =           GetBatteryLevel,
  .GetTemperature =            GetTemperatureLevel,
  .OnMacProcess =              OnMacProcessNotify,
  .OnJoinRequest =             OnJoinRequest,
  .OnTxData =                  OnTxData,
  .OnRxData =                  OnRxData
};

/**
  * @brief LoRaWAN handler parameters
  */
static LmHandlerParams_t LmHandlerParams =
{
  .ActiveRegion =             ACTIVE_REGION,
  .DefaultClass =             LORAWAN_DEFAULT_CLASS,
  .AdrEnable =                LORAWAN_ADR_STATE,
  .TxDatarate =               LORAWAN_DEFAULT_DATA_RATE,
  .PingPeriodicity =          LORAWAN_DEFAULT_PING_SLOT_PERIODICITY
};

/**
  * @brief Specifies the state of the application LED
  */
//static uint8_t AppLedStateOn = RESET;


/**
  * @brief Type of Event to generate application Tx
  */
static TxEventType_t EventType = TX_ON_TIMER;

/**
  * @brief Timer to handle the application Tx
  */
static UTIL_TIMER_Object_t TxTimer;

/**
  * @brief Timer to handle the application Tx Led to toggle
  */
static UTIL_TIMER_Object_t TxLedTimer;

/**
  * @brief Timer to handle the application Rx Led to toggle
  */
static UTIL_TIMER_Object_t RxLedTimer;

/**
  * @brief Timer to handle the application Join Led to toggle
  */
static UTIL_TIMER_Object_t JoinLedTimer;

/* USER CODE BEGIN PV */
static uint8_t Max_Co2_Msb = RESET;
static uint8_t Max_Co2_Lsb = RESET;
uint16_t Max_Co2;
bool Error_Flag = false;
uint8_t Max_Del_Abort = 48, counter;
uint8_t z;

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void)
{
  /* USER CODE BEGIN LoRaWAN_Init_1 */

  /* USER CODE END LoRaWAN_Init_1 */
  LED_Init(LED_BLUE);
  LED_Init(LED_RED1);
  LED_Init(LED_RED2);

  /* Get LoRa APP version*/
  if (DebugMode())
  {
	  APP_LOG(TS_OFF, VLEVEL_M, "APP_VERSION:        V%X.%X.%X\r\n",
	          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
	          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
	          (uint8_t)(__LORA_APP_VERSION >> __APP_VERSION_SUB2_SHIFT));

	  /* Get MW LoraWAN info */
	  APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION: V%X.%X.%X\r\n",
	          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_MAIN_SHIFT),
	          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB1_SHIFT),
	          (uint8_t)(__LORAWAN_VERSION >> __APP_VERSION_SUB2_SHIFT));

	  /* Get MW SubGhz_Phy info */
	  APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:   V%X.%X.%X\r\n",
	          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_MAIN_SHIFT),
	          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB1_SHIFT),
	          (uint8_t)(__SUBGHZ_PHY_VERSION >> __APP_VERSION_SUB2_SHIFT));
  }


  UTIL_TIMER_Create(&TxLedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&RxLedTimer, 0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnRxTimerLedEvent, NULL);
  UTIL_TIMER_Create(&JoinLedTimer, 0xFFFFFFFFU, UTIL_TIMER_PERIODIC, OnJoinTimerLedEvent, NULL);
  UTIL_TIMER_SetPeriod(&TxLedTimer, 500);
  UTIL_TIMER_SetPeriod(&RxLedTimer, 500);
  UTIL_TIMER_SetPeriod(&JoinLedTimer, 500);

  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU, LmHandlerProcess);
  UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), UTIL_SEQ_RFU, SendTxData);
  /* Init Info table used by LmHandler*/
  LoraInfo_Init();

  /* Init the Lora Stack*/
  LmHandlerInit(&LmHandlerCallbacks);

  LmHandlerConfigure(&LmHandlerParams);

  UTIL_TIMER_Start(&JoinLedTimer);

  LmHandlerJoin(ActivationType);

  if (EventType == TX_ON_TIMER)
  {
    /* send every time timer elapses */
    UTIL_TIMER_Create(&TxTimer,  0xFFFFFFFFU, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
    UTIL_TIMER_SetPeriod(&TxTimer,  APP_TX_DUTYCYCLE);
    UTIL_TIMER_Start(&TxTimer);
  }
  else
  {
    /* send every time button is pushed */
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
  }

  /* USER CODE BEGIN LoRaWAN_Init_Last */

  /* USER CODE END LoRaWAN_Init_Last */
}

void BSP_PB_Callback(Button_TypeDef Button)
{
  /* USER CODE BEGIN BSP_PB_Callback_1 */

  /* USER CODE END BSP_PB_Callback_1 */
  switch (Button)
  {
    case  BUTTON_USER:
      UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
      /* USER CODE BEGIN PB_Callback 1 */
      /* USER CODE END PB_Callback 1 */
      break;
    default:
      break;
  }
  /* USER CODE BEGIN BSP_PB_Callback_Last */

  /* USER CODE END BSP_PB_Callback_Last */
}

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params)
{
  /* USER CODE BEGIN OnRxData_1 */

  /* USER CODE END OnRxData_1 */
  if ((appData != NULL) && (params != NULL))
  {
    static const char *slotStrings[] = { "1", "2", "C", "C Multicast", "B Ping-Slot", "B Multicast Ping-Slot" };
    if (DebugMode())
    {
    	APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Indication ==========\r\n");
    }
    APP_LOG(TS_OFF, VLEVEL_H, "###### D/L FRAME:%04d | SLOT:%s | PORT:%d | DR:%d | RSSI:%d | SNR:%d\r\n",
            params->DownlinkCounter, slotStrings[params->RxSlot], appData->Port, params->Datarate, params->Rssi, params->Snr);
    switch (appData->Port)
    {
      case LORAWAN_SWITCH_CLASS_PORT:
        /*this port switches the class*/
        if (appData->BufferSize == 1)
        {
          switch (appData->Buffer[0])
          {
            case 0:
            {
              LmHandlerRequestClass(CLASS_A);
              break;
            }
            case 1:
            {
              LmHandlerRequestClass(CLASS_B);
              break;
            }
            case 2:
            {
              LmHandlerRequestClass(CLASS_C);
              break;
            }
            default:
              break;
          }
        }
        break;
      case LORAWAN_USER_APP_PORT:

          if (appData->BufferSize == 2)
          {
        	Max_Co2_Msb = appData-> Buffer[0];
        	Max_Co2_Lsb = appData-> Buffer[1];
        	Max_Co2 = (Max_Co2_Msb << 8) | Max_Co2_Lsb;
            if ((0x02BB < Max_Co2) && (Max_Co2 < 0x2001)) // 700 and 8192
            {
            	FLASHEx_EEPROM_WRITE(ADDBIAS_A, &Max_Co2_Msb, 1);
            	FLASHEx_EEPROM_WRITE(ADDBIAS_A + 0x01, &Max_Co2_Lsb, 1);
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);  // LED RX ON
                HAL_Delay(100);

                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);   // LED RX OFF
                HAL_Delay(100);
            }
            else
            {
            	for(z<6;z=0;z++){
                    HAL_GPIO_WriteToggle(GPIOA, GPIO_PIN_2);  // LED RX Blink 3 times
                    HAL_Delay(2000);
            	}

            	if (DebugMode())
            	{
            		APP_LOG(TS_OFF, VLEVEL_M, "\r\n Error \r\n", Max_Co2);
            	}
            }
          }
        break;
      /* USER CODE BEGIN OnRxData_Switch_case */

      /* USER CODE END OnRxData_Switch_case */
      default:
        /* USER CODE BEGIN OnRxData_Switch_default */

        /* USER CODE END OnRxData_Switch_default */
        break;
    }
  }

  /* USER CODE BEGIN OnRxData_2 */

  /* USER CODE END OnRxData_2 */
}

static void SendTxData(void)
{
//  uint16_t pressure = 0;
//  int16_t temperature = 0;
  sensor_t sensor_data;
  UTIL_TIMER_Time_t nextTxIn = 0;

#ifdef CAYENNE_LPP
  uint8_t channel = 0;
#else
//  uint16_t humidity = 0;

  uint32_t i = 0;
//  int32_t latitude = 0;
//  int32_t longitude = 0;
//  uint16_t altitudeGps = 0;
#endif /* CAYENNE_LPP */
  /* USER CODE BEGIN SendTxData_1 */
  uint16_t CO2 = 0;
  uint8_t SOC = 0;
  uint8_t SOH = 0;
  uint8_t ERROR = 0;
  EnvSensors_Read(&sensor_data);
  AppData.Port = LORAWAN_USER_APP_PORT;

  CO2 = (uint16_t)(sensor_data.CO2);
  SOC = (uint8_t)(sensor_data.SOC);
  SOH = (uint8_t)(sensor_data.SOH);

//  if (!((0x0190 < CO2) && (CO2 < 0x2000)))
//  {
//	  APP_LOG(TS_OFF, VLEVEL_M, "\r\n Error CO2 %u \r\n",CO2);
//	  Error_Flag = SetErrorFlags(1);
//  }
//  if (!((0x00 < SOC) && (SOC < 0x64)))
//  {
//	  APP_LOG(TS_OFF, VLEVEL_M, "\r\n Error SOC\r\n");
//	  Error_Flag = SetErrorFlags(2);
//  }
//  if (!((0x00 < SOH) && (SOH < 0x64)))
//  {
//	  APP_LOG(TS_OFF, VLEVEL_M, "\r\n Error SOH\r\n");
//	  Error_Flag = SetErrorFlags(3);
//  }


  FLASHEx_EEPROM_READ(ADDBIAS_F, &ERROR, 1);
  FLASHEx_EEPROM_READ(ADDBIAS_A, &Max_Co2_Msb, 1);
  FLASHEx_EEPROM_READ(ADDBIAS_A + 0x01, &Max_Co2_Lsb, 1);
  Max_Co2 = (Max_Co2_Msb << 8) | Max_Co2_Lsb;
  AppData.Buffer[i++] = (uint8_t)((CO2 >> 8) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(CO2 & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(SOC & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(SOH & 0xFF);
  AppData.Buffer[i++] = (uint8_t)((Max_Co2 >> 8) & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(Max_Co2 & 0xFF);
  AppData.Buffer[i++] = (uint8_t)(ERROR & 0xFF);
  AppData.BufferSize = i;

  /* USER CODE END SendTxData_1 */
//  EnvSensors_Read(&sensor_data);
//#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
//  temperature = (int16_t) sensor_data.temperature;
//#else
//  temperature = (SYS_GetTemperatureLevel() >> 8);
//#endif  /* SENSOR_ENABLED */
//  pressure    = (uint16_t)(sensor_data.pressure * 100 / 10);      /* in hPa / 10 */
//
//  AppData.Port = LORAWAN_USER_APP_PORT;
//
//#ifdef CAYENNE_LPP
//  CayenneLppReset();
//  CayenneLppAddBarometricPressure(channel++, pressure);
//  CayenneLppAddTemperature(channel++, temperature);
//  CayenneLppAddRelativeHumidity(channel++, (uint16_t)(sensor_data.humidity));
//
//  if ((LmHandlerParams.ActiveRegion != LORAMAC_REGION_US915) && (LmHandlerParams.ActiveRegion != LORAMAC_REGION_AU915)
//      && (LmHandlerParams.ActiveRegion != LORAMAC_REGION_AS923))
//  {
//    CayenneLppAddDigitalInput(channel++, GetBatteryLevel());
//    CayenneLppAddDigitalOutput(channel++, AppLedStateOn);
//  }
//
//  CayenneLppCopy(AppData.Buffer);
//  AppData.BufferSize = CayenneLppGetSize();
//#else  /* not CAYENNE_LPP */
//  humidity    = (uint16_t)(sensor_data.humidity * 10);            /* in %*10     */
//
//
//  if ((LmHandlerParams.ActiveRegion == LORAMAC_REGION_US915) || (LmHandlerParams.ActiveRegion == LORAMAC_REGION_AU915)
//      || (LmHandlerParams.ActiveRegion == LORAMAC_REGION_AS923))
//  {
//    AppData.Buffer[i++] = 0;
//    AppData.Buffer[i++] = 0;
//    AppData.Buffer[i++] = 0;
//    AppData.Buffer[i++] = 0;
//  }
//  else
//  {
//    latitude = sensor_data.latitude;
//    longitude = sensor_data.longitude;
//
//    AppData.Buffer[i++] = GetBatteryLevel();        /* 1 (very low) to 254 (fully charged) */
//    AppData.Buffer[i++] = (uint8_t)((latitude >> 16) & 0xFF);
//    AppData.Buffer[i++] = (uint8_t)((latitude >> 8) & 0xFF);
//    AppData.Buffer[i++] = (uint8_t)(latitude & 0xFF);
//    AppData.Buffer[i++] = (uint8_t)((longitude >> 16) & 0xFF);
//    AppData.Buffer[i++] = (uint8_t)((longitude >> 8) & 0xFF);
//    AppData.Buffer[i++] = (uint8_t)(longitude & 0xFF);
//    AppData.Buffer[i++] = (uint8_t)((altitudeGps >> 8) & 0xFF);
//    AppData.Buffer[i++] = (uint8_t)(altitudeGps & 0xFF);
//  }
//
//  AppData.BufferSize = i;
//#endif /* CAYENNE_LPP */

  if (LORAMAC_HANDLER_SUCCESS == LmHandlerSend(&AppData, LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, &nextTxIn, false))
  {
    APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\r\n");
  }
  else if (nextTxIn > 0)
  {
    APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\r\n", (nextTxIn / 1000));
  }
  /* USER CODE BEGIN SendTxData_2 */

  if (ERROR != 0X00) // reset ERROR FLAGS if no errors
  {
	  SetErrorFlags(5);
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // LED ON TX
  HAL_Delay(100);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);   // LED OFF TX
  HAL_Delay(100);

  if (CO2 > Max_Co2)
  {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // BUZZER ON
	  HAL_Delay(100);
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);   // BUZZER OFF
	  HAL_Delay(100);
	  if (!DebugMode())
	  {
		  GoToDeepSleep(300);      //  Sleep time in seconds
	  }
  }

  /* USER CODE END SendTxData_2 */
}

static void OnTxTimerEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerEvent_1 */

  /* USER CODE END OnTxTimerEvent_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

  /*Wait for next tx slot*/
  UTIL_TIMER_Start(&TxTimer);
  /* USER CODE BEGIN OnTxTimerEvent_2 */

  /* USER CODE END OnTxTimerEvent_2 */
}

static void OnTxTimerLedEvent(void *context)
{
  /* USER CODE BEGIN OnTxTimerLedEvent_1 */

  /* USER CODE END OnTxTimerLedEvent_1 */
  LED_Off(LED_RED2);
  /* USER CODE BEGIN OnTxTimerLedEvent_2 */

  /* USER CODE END OnTxTimerLedEvent_2 */
}

static void OnRxTimerLedEvent(void *context)
{
  /* USER CODE BEGIN OnRxTimerLedEvent_1 */

  /* USER CODE END OnRxTimerLedEvent_1 */
  LED_Off(LED_BLUE) ;
  /* USER CODE BEGIN OnRxTimerLedEvent_2 */

  /* USER CODE END OnRxTimerLedEvent_2 */
}

static void OnJoinTimerLedEvent(void *context)
{
  /* USER CODE BEGIN OnJoinTimerLedEvent_1 */

  /* USER CODE END OnJoinTimerLedEvent_1 */
  LED_Toggle(LED_RED1) ;
  /* USER CODE BEGIN OnJoinTimerLedEvent_2 */

  /* USER CODE END OnJoinTimerLedEvent_2 */
}

static void OnTxData(LmHandlerTxParams_t *params)
{
  /* USER CODE BEGIN OnTxData_1 */

  /* USER CODE END OnTxData_1 */
  if ((params != NULL) && (params->IsMcpsConfirm != 0))
  {
    LED_On(LED_RED2) ;
    UTIL_TIMER_Start(&TxLedTimer);
    if (DebugMode())
    {
        APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### ========== MCPS-Confirm =============\r\n");
    }
    APP_LOG(TS_OFF, VLEVEL_H, "###### U/L FRAME:%04d | PORT:%d | DR:%d | PWR:%d", params->UplinkCounter,
            params->AppData.Port, params->Datarate, params->TxPower);

    APP_LOG(TS_OFF, VLEVEL_H, " | MSG TYPE:");
    if (params->MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
    {
      APP_LOG(TS_OFF, VLEVEL_H, "CONFIRMED [%s]\r\n", (params->AckReceived != 0) ? "ACK" : "NACK");
    }
    else
    {
      APP_LOG(TS_OFF, VLEVEL_H, "UNCONFIRMED\r\n");
    }
  }

  /* USER CODE BEGIN OnTxData_2 */

  /* USER CODE END OnTxData_2 */
}

static void OnJoinRequest(LmHandlerJoinParams_t *joinParams)
{
  /* USER CODE BEGIN OnJoinRequest_1 */
uint8_t c = 0;
  /* USER CODE END OnJoinRequest_1 */
  if (joinParams != NULL)
  {
    if (joinParams->Status == LORAMAC_HANDLER_SUCCESS)
    {
      UTIL_TIMER_Stop(&JoinLedTimer);

      LED_Off(LED_RED1) ;
      if (DebugMode())
      {
    	  APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOINED = ");
      }
      if (joinParams->Mode == ACTIVATION_TYPE_ABP)
      {
    	if (DebugMode())
    	{
    		APP_LOG(TS_OFF, VLEVEL_M, "ABP ======================\r\n");
    	}

      }
      else
      {
    	if (DebugMode())
    	{
    		APP_LOG(TS_OFF, VLEVEL_M, "OTAA =====================\r\n");
    	}

      }
    }
    else
    {
    	if (DebugMode())
    	{
    		APP_LOG(TS_OFF, VLEVEL_M, "\r\n###### = JOIN FAILED\r\n");
    	}


		if (counter > 3)
		{
			counter = 0;
			FLASHEx_EEPROM_READ(ADDBIAS_A, &c, 1);
			c++;
			if (c >= Max_Del_Abort)           // FLAG MAX DELIVERIES ABORTED
			{
				APP_LOG(TS_OFF, VLEVEL_M, "\r\n Max. Deliveries Aborted reached. About to reset the device \r\n");
				Error_Flag = true;
				SetErrorFlags(4);
				c = 0x00;
				FLASHEx_EEPROM_WRITE(ADDBIAS_A, &c, 1);
				GoToDeepSleep(300);
			}
			else
			{
				FLASHEx_EEPROM_WRITE(ADDBIAS_A, &c, 1);
				APP_LOG(TS_OFF, VLEVEL_M, "\r\n Delivery aborted\r\n");
				if (!DebugMode())
				{
					GoToDeepSleep(300);    // Sleep time in seconds
				}
			}

		}

    }
  }

  /* USER CODE BEGIN OnJoinRequest_2 */

  /* USER CODE END OnJoinRequest_2 */
}

static void OnMacProcessNotify(void)
{
  /* USER CODE BEGIN OnMacProcessNotify_1 */

  /* USER CODE END OnMacProcessNotify_1 */
  UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);
  /* USER CODE BEGIN OnMacProcessNotify_2 */

  /* USER CODE END OnMacProcessNotify_2 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
