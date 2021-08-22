/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sys_sensors.c
  * @author  MCD Application Team
  * @brief   Manages the sensors on the application
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "sys_conf.h"
#include "sys_sensors.h"


#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
#if defined (X_NUCLEO_IKS01A2)
#include "iks01a2_env_sensors.h"
#elif defined (X_NUCLEO_IKS01A3)
#include "iks01a3_env_sensors.h"
#else  /* not X_IKS01xx */
#error "user to include its sensor drivers"
#endif  /* X_NUCLEO_IKS01xx */
#elif !defined (SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif  /* SENSOR_ENABLED */

/* USER CODE BEGIN Includes */
#include "stm32l0xx_hal.h"
#include "sys_app.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
#define STSOP_LATTITUDE           ((float) 43.618622 )  /*!< default latitude position */
#define STSOP_LONGITUDE           ((float) 7.051415  )  /*!< default longitude position */
#define MAX_GPS_POS               ((int32_t) 8388607 )  /*!< 2^23 - 1 */
#define HUMIDITY_DEFAULT_VAL      50.0f                 /*!< default humidity */
#define TEMPERATURE_DEFAULT_VAL   18.0f                 /*!< default temperature */
#define PRESSURE_DEFAULT_VAL      1000.0f               /*!< default pressure */

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
#if defined (X_NUCLEO_IKS01A2)
IKS01A2_ENV_SENSOR_Capabilities_t EnvCapabilities;
#elif defined (X_NUCLEO_IKS01A3)
IKS01A3_ENV_SENSOR_Capabilities_t EnvCapabilities;
#else  /* not X_IKS01Ax */
#error "user to include its sensor drivers"
#endif  /* X_NUCLEO_IKS01 */
#elif !defined (SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif  /* SENSOR_ENABLED */

/* USER CODE BEGIN PV */
I2C_HandleTypeDef hi2c1;
static const uint8_t CCS811_ADDR = 0x5A << 0x01;
static const uint8_t CCS811_MEAS_MODE = 0x01;
static const uint8_t CCS811_ALG_RESULT_DATA = 0x02;
static const uint8_t CCS811_HW_ID = 0x20;
static const uint8_t CCS811_SW_RESET = 0xFF;
static const uint8_t BQ27441_ADDR = 0x55 << 0x01;
static const uint8_t BQ27441_CONTROL = 0x00;
static const uint8_t BQ27441_DEVICE_TYPE = 0x01;
static const uint8_t BQ27441_FLAGS = 0x06;
static const uint8_t BQ27441_SOC = 0x1C;
static const uint8_t BQ27441_SOH = 0x20;
HAL_StatusTypeDef ret;
uint8_t buf[2];
UART_HandleTypeDef husart2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/
void EnvSensors_Read(sensor_t *sensor_data)
{
  /* USER CODE BEGIN EnvSensors_Read_1 */
	  uint16_t CO2_Value=0;
	  uint8_t SOC_Value=0;
	  uint8_t SOH_Value=0;

	  memset(buf, 0, sizeof(buf));
	  ret = HAL_I2C_Mem_Read( &hi2c1, CCS811_ADDR, CCS811_ALG_RESULT_DATA, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY );
	  CO2_Value = ((uint16_t)buf[0] << 8 | buf[1]);
	  memset(buf, 0, sizeof(buf));
	  ret = HAL_I2C_Mem_Read( &hi2c1, BQ27441_ADDR, BQ27441_SOC, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY );
	  SOC_Value = ((uint8_t)buf[0]);
	  memset(buf, 0, sizeof(buf));
	  ret = HAL_I2C_Mem_Read( &hi2c1, BQ27441_ADDR, BQ27441_SOH, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY );
	  SOH_Value = ((uint8_t)buf[0]);

  /* USER CODE END EnvSensors_Read_1 */
  float HUMIDITY_Value = HUMIDITY_DEFAULT_VAL;
  float TEMPERATURE_Value = TEMPERATURE_DEFAULT_VAL;
  float PRESSURE_Value = PRESSURE_DEFAULT_VAL;

#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
#if (USE_IKS01A2_ENV_SENSOR_HTS221_0 == 1)
  IKS01A2_ENV_SENSOR_GetValue(HTS221_0, ENV_HUMIDITY, &HUMIDITY_Value);
  IKS01A2_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
#endif /* USE_IKS01A2_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A2_ENV_SENSOR_LPS22HB_0 == 1)
  IKS01A2_ENV_SENSOR_GetValue(LPS22HB_0, ENV_PRESSURE, &PRESSURE_Value);
  IKS01A2_ENV_SENSOR_GetValue(LPS22HB_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
#endif /* USE_IKS01A2_ENV_SENSOR_LPS22HB_0 */
#if (USE_IKS01A3_ENV_SENSOR_HTS221_0 == 1)
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_HUMIDITY, &HUMIDITY_Value);
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_HTS221_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
#endif /* USE_IKS01A3_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A3_ENV_SENSOR_LPS22HH_0 == 1)
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_PRESSURE, &PRESSURE_Value);
  IKS01A3_ENV_SENSOR_GetValue(IKS01A3_LPS22HH_0, ENV_TEMPERATURE, &TEMPERATURE_Value);
#endif /* USE_IKS01A3_ENV_SENSOR_LPS22HH_0 */
#elif !defined (SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif  /* SENSOR_ENABLED */


  sensor_data->humidity    = HUMIDITY_Value;
  sensor_data->temperature = TEMPERATURE_Value;
  sensor_data->pressure    = PRESSURE_Value;

  sensor_data->latitude  = (int32_t)((STSOP_LATTITUDE  * MAX_GPS_POS) / 90);
  sensor_data->longitude = (int32_t)((STSOP_LONGITUDE  * MAX_GPS_POS) / 180);
  /* USER CODE BEGIN EnvSensors_Read_Last */
  sensor_data->CO2    = CO2_Value;
  sensor_data->SOC    = SOC_Value;
  sensor_data->SOH    = SOH_Value;
//  APP_LOG(TS_OFF, VLEVEL_M, "\r\n CO2 a %u\r\n",CO2_Value);
//  APP_LOG(TS_OFF, VLEVEL_M, "\r\n SOC b %u\r\n",SOC_Value);
//  APP_LOG(TS_OFF, VLEVEL_M, "\r\n SOH c %u\r\n",SOH_Value);
  /* USER CODE END EnvSensors_Read_Last */
}

void  EnvSensors_Init(void)
{
  /* USER CODE BEGIN EnvSensors_Init_1 */
// 	-- INIT CCS811 --
	 buf[0] = 0x11;
	 buf[1] = 0xE5;
	 buf[2] = 0x72;
	 buf[3] = 0x8A;
	 ret = HAL_I2C_Mem_Write( &hi2c1, CCS811_ADDR, CCS811_SW_RESET, I2C_MEMADD_SIZE_8BIT, buf, 4, HAL_MAX_DELAY );
	 HAL_Delay(100);
	 ret = HAL_I2C_Mem_Read( &hi2c1, CCS811_ADDR, CCS811_HW_ID, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY );
	 buf[0] = 0xF4;    // CCS811_APP_START
	 ret = HAL_I2C_Master_Transmit( &hi2c1, CCS811_ADDR, buf, 1, HAL_MAX_DELAY);
	 buf[0] = 0x00;
	 ret = HAL_I2C_Mem_Write( &hi2c1, CCS811_ADDR, CCS811_MEAS_MODE, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY );
	 buf[0] = 0x10;
	 ret = HAL_I2C_Mem_Write( &hi2c1, CCS811_ADDR, CCS811_MEAS_MODE, I2C_MEMADD_SIZE_8BIT, buf, 1, HAL_MAX_DELAY );

 // -- INIT BQ27411 -- This is done for a battery with a capacity of 1200 ma. In case of other capacity, please check
 //the bq27411-G1 Technical Reference SLUUAS7B (page 15)

 	 HAL_Delay(100);
 	 buf[0]=0x01;
 	 buf[1]=0x00;
 	 ret = HAL_I2C_Mem_Write( &hi2c1, BQ27441_ADDR, BQ27441_CONTROL, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY );
 	 ret = HAL_I2C_Mem_Read( &hi2c1, BQ27441_ADDR, BQ27441_CONTROL, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY );
 	 buf[0]=0x00;
 	 buf[1]=0x00;
 	 ret = HAL_I2C_Mem_Write( &hi2c1, BQ27441_ADDR, BQ27441_CONTROL, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY );
 	 ret = HAL_I2C_Mem_Read( &hi2c1, BQ27441_ADDR, BQ27441_CONTROL, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY );
 	 buf[0]=0x42;
 	 buf[1]=0x00;
 	 ret = HAL_I2C_Mem_Write( &hi2c1, BQ27441_ADDR, BQ27441_CONTROL, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY );
 	 ret = HAL_I2C_Mem_Read( &hi2c1, BQ27441_ADDR, BQ27441_FLAGS, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY );
 	 ret = HAL_I2C_Mem_Read( &hi2c1, BQ27441_ADDR, BQ27441_DEVICE_TYPE, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY );
  /* USER CODE END EnvSensors_Init_1 */

#if defined (SENSOR_ENABLED) && (SENSOR_ENABLED == 1)
  /* Init */
#if (USE_IKS01A2_ENV_SENSOR_HTS221_0 == 1)
  IKS01A2_ENV_SENSOR_Init(HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
#endif /* USE_IKS01A2_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A2_ENV_SENSOR_LPS22HB_0 == 1)
  IKS01A2_ENV_SENSOR_Init(LPS22HB_0, ENV_TEMPERATURE | ENV_PRESSURE);
#endif /* USE_IKS01A2_ENV_SENSOR_LPS22HB_0 */
#if (USE_IKS01A3_ENV_SENSOR_HTS221_0 == 1)
  IKS01A3_ENV_SENSOR_Init(IKS01A3_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);
#endif /* USE_IKS01A3_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A3_ENV_SENSOR_LPS22HH_0 == 1)
  IKS01A3_ENV_SENSOR_Init(IKS01A3_LPS22HH_0, ENV_TEMPERATURE | ENV_PRESSURE);
#endif /* USE_IKS01A3_ENV_SENSOR_LPS22HH_0 */

  /* Enable */
#if (USE_IKS01A2_ENV_SENSOR_HTS221_0 == 1)
  IKS01A2_ENV_SENSOR_Enable(HTS221_0, ENV_HUMIDITY);
  IKS01A2_ENV_SENSOR_Enable(HTS221_0, ENV_TEMPERATURE);
#endif /* USE_IKS01A2_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A2_ENV_SENSOR_LPS22HB_0 == 1)
  IKS01A2_ENV_SENSOR_Enable(LPS22HB_0, ENV_PRESSURE);
  IKS01A2_ENV_SENSOR_Enable(LPS22HB_0, ENV_TEMPERATURE);
#endif /* USE_IKS01A2_ENV_SENSOR_LPS22HB_0 */
#if (USE_IKS01A3_ENV_SENSOR_HTS221_0 == 1)
  IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_HUMIDITY);
  IKS01A3_ENV_SENSOR_Enable(IKS01A3_HTS221_0, ENV_TEMPERATURE);
#endif /* USE_IKS01A3_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A3_ENV_SENSOR_LPS22HH_0 == 1)
  IKS01A3_ENV_SENSOR_Enable(IKS01A3_LPS22HH_0, ENV_PRESSURE);
  IKS01A3_ENV_SENSOR_Enable(IKS01A3_LPS22HH_0, ENV_TEMPERATURE);
#endif /* USE_IKS01A3_ENV_SENSOR_LPS22HH_0 */

  /* Get capabilities */
#if (USE_IKS01A2_ENV_SENSOR_HTS221_0 == 1)
  IKS01A2_ENV_SENSOR_GetCapabilities(HTS221_0, &EnvCapabilities);
#endif /* USE_IKS01A2_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A2_ENV_SENSOR_LPS22HB_0 == 1)
  IKS01A2_ENV_SENSOR_GetCapabilities(LPS22HB_0, &EnvCapabilities);
#endif /* USE_IKS01A2_ENV_SENSOR_LPS22HB_0 */
#if (USE_IKS01A3_ENV_SENSOR_HTS221_0 == 1)
  IKS01A3_ENV_SENSOR_GetCapabilities(IKS01A3_HTS221_0, &EnvCapabilities);
#endif /* USE_IKS01A3_ENV_SENSOR_HTS221_0 */
#if (USE_IKS01A3_ENV_SENSOR_LPS22HH_0 == 1)
  IKS01A3_ENV_SENSOR_GetCapabilities(IKS01A3_LPS22HH_0, &EnvCapabilities);
#endif /* USE_IKS01A3_ENV_SENSOR_LPS22HH_0 */

#elif !defined (SENSOR_ENABLED)
#error SENSOR_ENABLED not defined
#endif /* SENSOR_ENABLED  */
  /* USER CODE BEGIN EnvSensors_Init_Last */

  /* USER CODE END EnvSensors_Init_Last */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
