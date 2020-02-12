/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  SRA - Central Labs
  * @version v2.1.0
  * @date    5-Apr-2019
  * @brief   Initialization of the Target Platform
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"

/* Imported variables ---------------------------------------------------------*/
extern USBD_DescriptorsTypeDef VCP_Desc;

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

USBD_HandleTypeDef  USBD_Device;

/* Local defines -------------------------------------------------------------*/



/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);

/**
  * @brief  Initialize all the Target platform's Features
  * @param  TargetType_t BoardType Nucleo/BlueCoin/SensorTile
  * @retval None
  */
void InitTargetPlatform(TargetType_t BoardType)
{
  TargetBoardFeatures.BoardType = BoardType;

#ifdef ENABLE_USB_DEBUG
  /* enable USB power on Pwrctrl CR2 register */
  HAL_PWREx_EnableVddUSB();

  /* Configure the CDC */
  /* Init Device Library */
  USBD_Init(&USBD_Device, &VCP_Desc, 0);
  /* Add Supported Class */
  USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
  /* Add Interface callbacks for AUDIO and CDC Class */
  USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
  /* Start Device Process */
  USBD_Start(&USBD_Device);
  /* 10 seconds ... for having time to open the Terminal
   * for looking the MOTENV1 Initialization phase */
  HAL_Delay(10000);
#endif /* ENABLE_USB_DEBUG */
  
  /* Initialize LED */
  BSP_LED_Init( LED1 );

  STLBLE_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
         "\tSensorTile"
          "\r\n",
          STLBLE_PACKAGENAME,
          STLBLE_VERSION_MAJOR,STLBLE_VERSION_MINOR,STLBLE_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the Target's Features */
  Init_MEM1_Sensors();
  
  TargetBoardFeatures.LedStatus = 0; /*Off by default */
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
  /* Humidity Temperature1 */
  if(BSP_ENV_SENSOR_Init(HTS221_0, ENV_HUMIDITY|ENV_TEMPERATURE)==BSP_ERROR_NONE)
  {
    STLBLE_PRINTF("OK Humidity Sensor\n\r");
    TargetBoardFeatures.HandleHumSensor=1;

    STLBLE_PRINTF("OK Temperature Sensor1\n\r");
    TargetBoardFeatures.NumTempSensors++;
    TargetBoardFeatures.HandleTempSensors[0]=1;
  }

  /* Temperature2 Pressure */
  if(BSP_ENV_SENSOR_Init(LPS22HB_0, ENV_TEMPERATURE|ENV_PRESSURE)==BSP_ERROR_NONE)
  {
     STLBLE_PRINTF("OK Temperature Sensor2\n\r");
     TargetBoardFeatures.NumTempSensors++;
     TargetBoardFeatures.HandleTempSensors[1]=1;

     STLBLE_PRINTF("OK Pressure Sensor\n\r");
     TargetBoardFeatures.HandlePressSensor=1;
  }

  if(TargetBoardFeatures.HandleHumSensor) {
    if(BSP_ENV_SENSOR_Enable(HTS221_0, ENV_HUMIDITY)==BSP_ERROR_NONE)
    {
      STLBLE_PRINTF("Enabled Humidity Sensor\n\r");
    }
  }
  
  if(TargetBoardFeatures.HandleTempSensors[0]){
    if(BSP_ENV_SENSOR_Enable(HTS221_0, ENV_TEMPERATURE)==BSP_ERROR_NONE)
    {
      STLBLE_PRINTF("Enabled Temperature Sensor1\n\r");
    }
  }
  
  if(TargetBoardFeatures.HandleTempSensors[1]){
    if(BSP_ENV_SENSOR_Enable( LPS22HB_0, ENV_TEMPERATURE)==BSP_ERROR_NONE) 
    {
      STLBLE_PRINTF("Enabled Temperature Sensor2\n\r");
    }
  }
  
  if(TargetBoardFeatures.HandlePressSensor) {
    if(BSP_ENV_SENSOR_Enable(LPS22HB_0, ENV_PRESSURE)==BSP_ERROR_NONE)
    {
      STLBLE_PRINTF("Enabled Pressure Sensor\n\r");
    }
  }
}

/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  BSP_LED_On( LED1 );
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off( LED1 );
}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
