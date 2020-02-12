/**
******************************************************************************
* @file    main.c
* @author  SRA - Central Labs
* @version v2.1.0
* @date    5-Apr-2019
* @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_interface.h"
#include "SensorTile.h"


/*NEW*/
#include "hal_common.h"
#include "bluenrg_mesh.h"
#include "appli_mesh.h"
#include "appli_light.h"
#include "models_if.h"
#include "mesh_cfg.h"
#include "cube_hal.h"
/*END_NEW*/

// si le Macro SERVER_ROLE est defini, il est serveur,
// si non, il est client
//#define SERVER_ROLE

/* Private typedef -----------------------------------------------------------*/
#define STATIC_BLE_MAC

/* Private define ------------------------------------------------------------*/
USBD_HandleTypeDef  USBD_Device;
void HAL_PWREx_EnableVddUSB();
void HAL_PWREx_EnableVddIO2();

/* Imported Variables -------------------------------------------------------------*/
extern TIM_HandleTypeDef TimHandle;
extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* Exported Variables -------------------------------------------------------------*/
uint32_t ConnectionBleStatus  =0;
uint8_t BufferToWrite[256];
int32_t BytesToWrite;
TIM_HandleTypeDef TimCCHandle;

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t SendEnv = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);
static void InitTimers(void);
static void SendEnvironmentalData(void);

#define BDADDR_SIZE 6
uint8_t bnrg_expansion_board = IDB05A1;

#ifdef SERVER_ROLE
  BLE_RoleTypeDef BLE_Role = SERVER;
#else
  BLE_RoleTypeDef BLE_Role = CLIENT;
#endif

int static counter_send_test; // counter for send test message to server, in function User_Process()
extern volatile uint8_t set_connectable;
extern volatile int     connected;
extern volatile uint8_t notification_enabled;
extern volatile uint8_t end_read_tx_char_handle;
extern volatile uint8_t end_read_rx_char_handle;
static void MX_BlueNRG_MS_Init(void);
static void User_Process(void);

/*-*-*-*-*-*-*-*-*-*-*-*-*NEW*-*-*-*-*-*-*-*-*-*-*-*-*-*/
const MOBLE_USER_BLE_CB_MAP user_ble_cb =
{
  Appli_BleStackInitCb,
  Appli_BleSetTxPowerCb,
  Appli_BleGattConnectionCompleteCb,
  Appli_BleGattDisconnectionCompleteCb,
  Appli_BleUnprovisionedIdentifyCb,
  Appli_BleSetUUIDCb,
  Appli_BleSetProductInfoCB,
  Appli_BleSetNumberOfElementsCb,
  Appli_BleDisableFilterCb
};
//
//tr_params_t TrParams1 = {
//		500U
//};
//fn_params_t FnParams1 = {
//		1U
//};
//lpn_params_t LpnParams1 = {
//		1U,
//		1U,
//		2U,
//		150U,
//		2000U,
//		50U,
//		25U,
//		55U,
//		2U,
//		-100U,
//		10U
//};
//
//neighbor_table_init_params_t NeighborTableParams1 = {
//		5U,
//		20U,
//		1U,
//		0U,
//		0U
//};
//#define BLUENRG_MESH_FEATURES1 (1 << 0 | 1 << 1 | 0 << 2 )
//#define BLUENRG_MESH_PRVN_BEARER_INFO1 (1 << 0 | 0 << 1)
//
//prvn_params_t PrvnParams1 = {
//		FALSE,
//		{0xF4, 0x65, 0xE4, 0x3F, 0xF2, 0x3D, 0x3F, 0x1B, 0x9D, 0xC7,
//		0xDF, 0xC0, 0x4D, 0xA8, 0x75, 0x81, 0x84, 0xDB, 0xC9, 0x66, 0x20, 0x47, 0x96, 0xEC, 0xCF,
//		0x0D, 0x6C, 0xF5, 0xE1, 0x65, 0x00, 0xCC, 0x02, 0x01, 0xD0, 0x48, 0xBC, 0xBB, 0xD8, 0x99,
//		0xEE, 0xEF, 0xC4, 0x24, 0x16, 0x4E, 0x33, 0xC2, 0x01, 0xC2, 0xB0, 0x10, 0xCA, 0x6B, 0x4D,
//		0x43, 0xA8, 0xA1, 0x55, 0xCA, 0xD8, 0xEC, 0xB2, 0x79},
//		{0x52, 0x9A, 0xA0, 0x67, 0x0D, 0x72, 0xCD, 0x64, 0x97, 0x50,
//		0x2E, 0xD4, 0x73, 0x50, 0x2B, 0x03, 0x7E, 0x88, 0x03, 0xB5, 0xC6, 0x08, 0x29, 0xA5, 0xA3,
//		0xCA, 0xA2, 0x19, 0x50, 0x55, 0x30, 0xBA},
//		16U,
//		{0},
//		OUTPUT_OOB_ACTION_BIT_BLINK,
//		0U,
//		Appli_BleOutputOOBAuthCb,
//		INPUT_OOB_ACTION_BIT_PUSH,
//		0U,
//		Appli_BleInputOOBAuthCb
//};
//
//MOBLEUINT8 dyn_buffer_m1[7000U] = {0};
//
//DynBufferParam_t DynBufferParam1 = {
//		dyn_buffer_m1,
//		1000U,
//		1U,
//		160U,
//		0U,
//};

//const Mesh_Initialization_t BLEMeshlib_Init_params1 = {
//		&bdaddr,
//		  &TrParams1,//
//		  &FnParams1,
//		  &LpnParams1,
//		  &NeighborTableParams1,//
//		  BLUENRG_MESH_FEATURES1,
//		  BLUENRG_MESH_PRVN_BEARER_INFO1,//
//		  &PrvnParams1,//
//		  &DynBufferParam1
//};


const Mesh_Initialization_t BLEMeshlib_Init_params = {
  bdaddr,
  &TrParams,//
  &FnParams,
  &LpnParams,
  &NeighborTableParams,//
  BLUENRG_MESH_FEATURES,
  BLUENRG_MESH_PRVN_BEARER_INFO,//
  &PrvnParams,//
  &DynBufferParam
};

void clignote_fault(void){
	LedOffTargetPlatform();
	HAL_Delay(100);
	LedOnTargetPlatform();
	HAL_Delay(100);

}

/**********************************************************************************/
int main(void)
{
// DEBUT D'INITIALISATION
  uint32_t StartTime;
  MOBLE_RESULT status_Mesh;
  HAL_Init();
  /* Configure the System clock */
  SystemClock_Config();
  InitTargetPlatform(TARGET_SENSORTILE);
  HAL_FLASH_Unlock();
  __HAL_RCC_CRC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
// FIN D'INITIALISATION

  STLBLE_PRINTF("\t(HAL %ld.%ld.%ld_%ld)\r\n"
                "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
                  " (IAR)\r\n"
#elif defined (__CC_ARM)
                    " (KEIL)\r\n"
#elif defined (__GNUC__)
                      " (openstm32)\r\n"
#endif
		"\tSend Every %4dmS Temperature/Humidity/Pressure\r\n",
		HAL_GetHalVersion() >>24,
		(HAL_GetHalVersion() >>16)&0xFF,
		(HAL_GetHalVersion() >> 8)&0xFF,
		HAL_GetHalVersion()      &0xFF,
		__DATE__,__TIME__,
		uhCCR1_Val/10);

  STLBLE_PRINTF("***************** COMMENCER CONSTRUIRE MESH**************\r\n");

  if (!Appli_CheckBdMacAddr())
    {
	  STLBLE_PRINTF("Etape1:  Bad BD_MAC ADDR!\r\n");
      /* LED Blinks if BDAddr is not appropriate */
      while (1)
      {
    	  clignote_fault();
      }
    }
  else {
	  STLBLE_PRINTF("Etape1:  Good BD_MAC ADDR!\r\n");
  }

  /* Set BLE configuration function callbacks */
  status_Mesh =  BluenrgMesh_BleHardwareInitCallBack(&user_ble_cb);
  STLBLE_PRINTF("BlueMesh Status0 BluenrgMesh_BleHardwareInitCallBack:  %d \r\n", status_Mesh);

    /* Initializes BlueNRG-Mesh Library */
    STLBLE_PRINTF("Etape2 - BoardMAC = %x:%x:%x:%x:%x:%x\r\n",bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);
    STLBLE_PRINTF("BlueNRG-Mesh Lighting Demo v%s\n\r", BLUENRG_MESH_APPLICATION_VERSION);
    STLBLE_PRINTF("BlueNRG-Mesh Library v%s\n\r", BluenrgMesh_GetLibraryVersion());
    STLBLE_PRINTF("BlueNRG-Mesh Library subversion v%s\n\r", BluenrgMesh_GetLibrarySubVersion());
    STLBLE_PRINTF("BD_MAC Address = [%02x]:[%02x]:[%02x]:[%02x]:[%02x]:[%02x] \n\r",
                bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);
    /*    PAS DE PROBLEME JUSQU'A ICI      */

	  STLBLE_PRINTF("BlueMesh Status BluenrgMesh_Init:  %d \r\n", BluenrgMesh_Init(&BLEMeshlib_Init_params));

	  /* Initializes BlueNRG-Mesh Library */
	    if (MOBLE_FAILED (BluenrgMesh_Init(&BLEMeshlib_Init_params) ))
	    {
	    	STLBLE_PRINTF("Could not initialize BlueNRG-Mesh library!\r\n");
	      /* LED continuously blinks if library fails to initialize */
	      while (1)
	      {
	        Appli_LedBlink();
	      }
	    }

	    /* Checks if the node is already provisioned or not */
	    if (BluenrgMesh_IsUnprovisioned() == MOBLE_TRUE)
	    {
	      /*  BluenrgMesh_UnprovisionedNodeInfo(&UnprovNodeInfoParams); */
	        BluenrgMesh_InitUnprovisionedNode(); /* Initalizes Unprovisioned node */

	        printf("Unprovisioned device \r\n");
	  #if PB_ADV_SUPPORTED
	      BluenrgMesh_SetUnprovisionedDevBeaconInterval(100);
	  #endif
	    }
	    else
	    {
	      BluenrgMesh_InitProvisionedNode();  /* Initalizes Provisioned node */
	      printf("Provisioned node \r\n");
	    }

	    /* Initializes the Application */
	    /* This function also checks for Power OnOff Cycles
	       Define the following Macro "ENABLE_UNPROVISIONING_BY_POWER_ONOFF_CYCLE"
	       to check the Power-OnOff Cycles
	      5 Continous cycles of OnOff with Ontime <2 sec will cause unprovisioning
	    */
	  	Appli_Init();

	  /* Check to manually unprovision the board */
	    Appli_CheckForUnprovision();
	    /* Set attention timer callback */
	    BluenrgMesh_SetAttentionTimerCallback(Appli_BleAttentionTimerCb);

	  /* Models intialization */
	    BluenrgMesh_ModelsInit();

	    /* Main infinite loop */
	    while(1)
	    {
	      BluenrgMesh_Process();
	      BluenrgMesh_ModelsProcess(); /* Models Processing */
	      Appli_Process();
	    }

}


/**
* @brief  Output Compare callback in non blocking mode
* @param  htim : TIM OC handle
* @retval None
*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;

  /* TIM1_CH1 toggling with frequency = 2Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    SendEnv=1;
  }
}

/**
* @brief  Period elapsed callback in non blocking mode for Environmental timer
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimHandle))
  {
    CDC_TIM_PeriodElapsedCallback(htim);
  }
}

/**
* @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
* @param  None
* @retval None
*/
static void SendEnvironmentalData(void)
{
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
  {
    BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else
  {
    STLBLE_PRINTF("Sending: ");
  }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */

  /* Pressure,Humidity, and Temperatures*/
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV))
  {
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;

    if(TargetBoardFeatures.HandlePressSensor)
    {
      BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_PRESSURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
      PressToSend=intPart*100+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        STLBLE_PRINTF("Press=%ld ",PressToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
    }

    if(TargetBoardFeatures.HandleHumSensor)
    {

      BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_HUMIDITY, (float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      HumToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        STLBLE_PRINTF("Hum=%d ",HumToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
    }

    if(TargetBoardFeatures.NumTempSensors==2)
    {
      BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp1ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        STLBLE_PRINTF("Temp=%d ",Temp1ToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */

      BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_TEMPERATURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp2ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        STLBLE_PRINTF("Temp2=%d ",Temp2ToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
    }
    else if(TargetBoardFeatures.NumTempSensors==1)
    {
      if (BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE,(float *)&SensorValue)!=BSP_ERROR_NONE)
      {
        BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_TEMPERATURE,(float *)&SensorValue);
      }
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp1ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp1=%d ",Temp1ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        STLBLE_PRINTF("Temp1=%d ",Temp1ToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */

    }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  }

#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
  {
    BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else
  {
    STLBLE_PRINTF("\r\n");
  }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
}

/**
* @brief  Function for initializing timers for sending the information to BLE:
*  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
*  - 1 for sending the Environmental info
* @param  None
* @retval None
*/
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;

  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;

  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1);

  /* Set TIM1 instance (Motion)*/
  /* Set TIM1 instance */
  TimCCHandle.Instance = TIM1;
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Configure the Output Compare channels */
  /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;

  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }

}

/** @brief Initialize the BlueNRG Stack
* @param None
* @retval None
*/
static void Init_BlueNRG_Stack(void)
{
  const char BoardName[8] = {NAME_STLBLE,0};
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;

#ifdef STATIC_BLE_MAC
  {
    uint8_t tmp_bdaddr[6]= {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* STATIC_BLE_MAC */

  /* Initialize the BlueNRG HCI */
  hci_init(HCI_Event_CB, NULL);

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /*
  * Reset BlueNRG again otherwise we won't
  * be able to change its MAC address.
  * aci_hal_write_config_data() must be the first
  * command after reset otherwise it will fail.
  */
  hci_reset();

  HAL_Delay(100);

#ifndef STATIC_BLE_MAC
  /* Create a Unique BLE MAC */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
    bdaddr[4] = (((STLBLE_VERSION_MAJOR-48)*10) + (STLBLE_VERSION_MINOR-48)+100)&0xFF;
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
#else /* STATIC_BLE_MAC */

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret)
  {
    STLBLE_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
    goto fail;
  }
#endif /* STATIC_BLE_MAC */

  ret = aci_gatt_init();
  if(ret)
  {
    STLBLE_PRINTF("\r\nGATT_Init failed\r\n");
    goto fail;
  }


    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if(ret != BLE_STATUS_SUCCESS)
  {
    STLBLE_PRINTF("\r\nGAP_Init failed\r\n");
    goto fail;
  }

#ifndef  STATIC_BLE_MAC
  ret = hci_le_set_random_address(bdaddr);

  if(ret)
  {
    STLBLE_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
    goto fail;
  }
#endif /* STATIC_BLE_MAC */

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);

  if(ret)
  {
    STLBLE_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS)
  {
    STLBLE_PRINTF("\r\nGAP setting Authentication failed\r\n");
    goto fail;
  }

  STLBLE_PRINTF("SERVER: BLE Stack Initialized \r\n"
                "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
                  "\t\tBoardName= %s\r\n"
                    "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
                    "SensorTile",
                    hwVersion,
                    fwVersion>>8,
                    (fwVersion>>4)&0xF,
                    (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
                    BoardName,
                    bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);

  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);

  return;

fail:
  return;
}

/** @brief Initialize all the Custom BlueNRG services
* @param None
* @retval None
*/
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;

  ret = Add_HWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
    STLBLE_PRINTF("HW      Service W2ST added successfully\r\n");
  }
  else
  {
    STLBLE_PRINTF("\r\nError while adding HW Service W2ST\r\n");
  }

  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS)
  {
    STLBLE_PRINTF("Config  Service W2ST added successfully\r\n");
  }
  else
  {
    STLBLE_PRINTF("\r\nError while adding Config Service W2ST\r\n");
  }
}

/**
* @brief  System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }

  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();

  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }

  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();

  /* Select MSI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    while(1);
  }
}


/**
* @brief This function provides accurate delay (in milliseconds) based
*        on variable incremented.
* @note This is a user implementation using WFI state
* @param Delay: specifies the delay time length, in milliseconds.
* @retval None
*/
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay)
  {
    __WFI();
  }
}


/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {

  }
}



#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: STLBLE_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1){
  }
}
#endif
static void MX_BlueNRG_MS_Init(void)
{
  /* Initialize the peripherals and the BLE Stack */
	const char BoardName[8] = {NAME_STLBLE,0};
  uint8_t CLIENT_BDADDR[] = {0xbb, 0x00, 0x00, 0xE1, 0x80, 0x02};
  uint8_t SERVER_BDADDR[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;

  uint8_t  hwVersion;
  uint16_t fwVersion;
  int ret;

  /* Get the User Button initial state */

  hci_init(user_notify, NULL);  /*IMPORTANT-new-handle*/

  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /*
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  hci_reset();

  HAL_Delay(100);

  bnrg_expansion_board = IDB05A1;

  if (BLE_Role == CLIENT) {
    BLUENRG_memcpy(bdaddr, CLIENT_BDADDR, sizeof(CLIENT_BDADDR));
    STLBLE_PRINTF("I-1. Client address copied!! \r\n");
  } else {
    BLUENRG_memcpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));
    STLBLE_PRINTF("I-1. Server address copied!! \r\n");
  }

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if (ret) {
    STLBLE_PRINTF("Setting BD_ADDR failed 0x%02x.\n", ret);
  } else {
	  STLBLE_PRINTF("I-2. aci_hal_write_config_data successed\r\n");
  }

  ret = aci_gatt_init();
  if (ret) {
	  STLBLE_PRINTF("GATT_Init failed.\n");
  }else {
	  STLBLE_PRINTF("I-3. aci_gatt_init successed\r\n");
  }

  if (BLE_Role == SERVER) {
	  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	  STLBLE_PRINTF("I-4. Server aci_gap_init_IDB05A1 successed\r\n");
  }
  else {
      ret = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	  STLBLE_PRINTF("I-4. Client aci_gap_init_IDB05A1 successed\r\n");
  }

  if (ret != BLE_STATUS_SUCCESS) {
	  STLBLE_PRINTF("I-4.GAP_Init failed.\n");
  }

  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret == BLE_STATUS_SUCCESS) {
	  STLBLE_PRINTF("I-5. aci_gap_set_auth_requirement successed! \r\n");
  }

  STLBLE_PRINTF("***********SUMMARY*****PHASE_I****\r\n");
  STLBLE_PRINTF("SERVER: BLE Stack Initialized \r\n"
                  "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
                    "\t\tBoardName= %s\r\n"
                      "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
                      "SensorTile",
                      hwVersion,
                      fwVersion>>8,
                      (fwVersion>>4)&0xF,
                      (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
                      BoardName,
                      bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);




  // ADD SERVICE!!!!
  if (BLE_Role == SERVER) {
	STLBLE_PRINTF("I-6. SERVER: BLE Stack Initialized\r\n");
    ret = Add_Sample_Service();

    if (ret == BLE_STATUS_SUCCESS){
    	STLBLE_PRINTF("I-7. Sample_Service added successfully!\r\n");
    }
    else {
    	STLBLE_PRINTF("Error while adding service.\n");
    }

  } else {
	  STLBLE_PRINTF("CLIENT: BLE Stack Initialized\n");
  }



  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

  /* USER CODE BEGIN BlueNRG_MS_Init_PostTreatment */

  /* USER CODE END BlueNRG_MS_Init_PostTreatment */
}


static void User_Process(void){
	if (set_connectable)
	  {
	    /* Establish connection with remote device */
	    Make_Connection();
	    STLBLE_PRINTF("II-1. Make_Connection successed! \r\n");
	    set_connectable = FALSE;
	  }


	  if (BLE_Role == CLIENT)
	  {
	    /* Start TX handle Characteristic dynamic discovery if not yet done */
	    if (connected && !end_read_tx_char_handle){
	    	STLBLE_PRINTF("II-2-1. connected && !end_read_tx \r\n");
	    	startReadTXCharHandle();
	    }
	    /* Start RX handle Characteristic dynamic discovery if not yet done */
	    else if (connected && !end_read_rx_char_handle){
	    	STLBLE_PRINTF("II-2-2. connected && !end_read_rx \r\n");
	        startReadRXCharHandle();
	    }

	    if (connected && end_read_tx_char_handle && end_read_rx_char_handle && !notification_enabled)
	    {
	      //BSP_LED_Off(LED2); //end of the connection and chars discovery phase
	    	STLBLE_PRINTF("II-2-3. connected && end_read_rx && end_read_tx && !notific \r\n");
	        enableNotification();
	    }



	    /* Send a toggle command to the remote device */
		  if (counter_send_test > 500){
			  counter_send_test = 0;
			  //STLBLE_PRINTF("**\r\n");
			  HAL_Delay(50);
			  if (connected && notification_enabled)
			  {
				  /*!Envoyer des donnee!*/
				  float SensorValue;
				  int16_t Temp1ToSend=0;
				  int32_t decPart, intPart;

					if (BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE,(float *)&SensorValue)!=BSP_ERROR_NONE)
					{
					  BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_TEMPERATURE,(float *)&SensorValue);
					}
					MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
					Temp1ToSend = intPart*10+decPart;



				  uint8_t data[20] = "Tempera_recv : "; //= "0123456789";//{'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J'};
				  uint8_t chiffre[5];
				  itoa(Temp1ToSend, chiffre, 10);
				  strcat(data, chiffre);

				  sendData(data, sizeof(data));
				  //STLBLE_PRINTF("data sent successfully \r\n");
				  STLBLE_PRINTF("Temperature sent : %s \r\n", chiffre);
				  data[20] = "0";
				  HAL_Delay(50);
				  }
		  } else {
			  counter_send_test ++;
		  }
	  }



}

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
