/**
******************************************************************************
* @file    appli_nvm.c
* @author  BLE Mesh Team
* @version V1.3.0
* @date    19-09-2019
* @brief   Application interface for NVM = Non Volatile memory interface
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
* Initial BlueNRG-Mesh is built over Motorola�s Mesh over Bluetooth Low Energy 
* (MoBLE) technology. The present solution is developed and maintained for both 
* Mesh library and Applications solely by STMicroelectronics.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "hal_common.h"
#include "appli_nvm.h"
#include "mesh_cfg.h"
#include "pal_nvm.h"
#include "appli_mesh.h"
#include "common.h"
#include "main.h"

MOBLE_RESULT status_Mesh;

extern const MOBLEUINT8* _bdaddr[];
extern const void* mobleNvmBase;
//extern const void* bnrgmNvmBase;
extern MOBLEUINT8 PowerOnOff_flag;
/* Reserved for Bluenrg-Mesh library */
#define BLUENRGMESH_NVM_BASE               ((unsigned int)mobleNvmBase)
#define BLUENRGMESH_NVM_PAGE               2U //NEW /* Library parameter to be saved in 2nd page of 2nd bank of STM32L4*/
#define BLUENRGMESH_NVM_BACKUP_BASE        (BLUENRGMESH_NVM_BASE + PAGE_SIZE)


#define APP_NVM_BASE                      ((unsigned int)_bdaddr)
#define APP_NVM_SIZE                      2048U
#define APP_NVM_RESERVED_SIZE             256U
#define APP_NVM_RESERVED_MODEL_SIZE       2048U  /* Next Page of APP_NVM_BASE*/
#define APP_NVM_SUBPAGE_SIZE              256U
#define APP_NVM_MAX_SUBPAGE               15U
#define APP_NVM_SUBPAGE_OFFSET(i)         (unsigned int)(APP_NVM_BASE+APP_NVM_RESERVED_SIZE+256*i)
#define MODEL_PAGE_NUMBER		  1U /* Model data is written in next page of Application NVM base address */
#define RESET_PAGE_NUMBER	       	  0 /* RESET count is written in NVM base address */

#define APP_NVM_RESET_COUNT_SIZE           16U /* 12 bytes reserved for write cycle optimization */
#define APP_NVM_GENERIC_MODEL_OFFSET      (unsigned int)(APP_NVM_RESERVED_MODEL_SIZE)   /* Model data are save in page next to the Bdaddr page
                                                                                            ie 2nd page of Bank 2*/
#define APP_NVM_LIGHT_MODEL_OFFSET        (unsigned int)(APP_NVM_GENERIC_MODEL_OFFSET + APP_NVM_GENERIC_MODEL_SIZE)

/* Private variables ---------------------------------------------------------*/

/**
* @brief  This function erases a specified flash page
* @param  Page Number
* @retval MOBLE_RESULT_INVALIDARG if PageNumber overflow
*         MOBLE_RESULT_FALSE if flash operation is very close to next radio event
*         MOBLE_RESULT_FAIL if flash command error is set
*         MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT AppliNvm_FlashErase(uint16_t PageNumber)
{
  MOBLE_RESULT result;
  
  if (PageNumber < N_PAGES && PageNumber!= BLUENRGMESH_NVM_PAGE)
  {
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_OPTVERR);

    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_2;
    erase.Page = PageNumber; 
    erase.NbPages = 1;

    uint32_t pageError = 0;
    
    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &pageError);

    HAL_FLASH_Lock();

    if (status == HAL_OK)
      result = MOBLE_RESULT_SUCCESS;
    else
      result = MOBLE_RESULT_FAIL;
  }
  else /* Invalid page no */
  {
    result = MOBLE_RESULT_INVALIDARG;
  }
  return result;   
}

/** 
*  @brief  Program Flash using FLASH_WRITE or FLASH_BURSTWRITE
*          User to make sure that programming word burst should not happen across multiple pages
*  @param  Address (word aligned)
*  @param  Data to be programmed
*  @param  size of data to be programmed
* @retval  MOBLE_RESULT_INVALIDARG if parameters invalid
*          MOBLE_RESULT_FALSE if flash operation is very close to next radio event
*          MOBLE_RESULT_FAIL if flash command error is set
*          MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT AppliNvm_FlashProgram(uint32_t Address, uint8_t Data[], uint16_t size)
{	
  MOBLE_RESULT result = MOBLE_RESULT_SUCCESS;

  
//    if (Address > APP_NVM_SIZE)
//    {
//        result = MOBLE_RESULT_INVALIDARG;
//    }
     if (size == 0)
      {
        result = MOBLE_RESULT_FALSE;
      }
//    else if (Address + size > APP_NVM_SIZE)
//    {
//        result = MOBLE_RESULT_INVALIDARG;
//    }
//    else if (Address & 3)
//    {
//        result = MOBLE_RESULT_INVALIDARG;
//    }
//    else if (size & 3)
//    {
//        result = MOBLE_RESULT_INVALIDARG;
//    }
      else
      {
      size >>= 3;
      size +=1;
     
      uint64_t* src = (uint64_t*)Data;
      uint64_t* dst = (uint64_t*)(Address);
      
      HAL_StatusTypeDef status = HAL_OK;

      HAL_FLASH_Unlock();
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_OPTVERR);
      
      for (size_t i = 0; i < size && status == HAL_OK; ++i)
        {
            if (src[i] != dst[i])
            {
#if !defined(DISABLE_TRACES)
              //printf("W[%.8x]: 0x%.8x\r\n", Address + (i << 3), (uint64_t)src[i]);
#endif
                status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address + (i << 3), (uint64_t)src[i]);
                
                if (status != HAL_OK)
                {
                    break;
                }
            }
        }
       HAL_FLASH_Lock();

        if (HAL_OK != status)
        {
            result = MOBLE_RESULT_FAIL;
        }
    } 
  
  return result;
}


/**
*  @brief  Mark first valid subpage as invalid. 
*  @param  void
*  @retval MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT AppliNvm_MarkSubpageInvalid(void)
{
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  This function writes Reset count value in flash
*         if device has been turned ON 5 times consecutively with each ON duration < 2s, reset to factory settings
*         LED blinks 2 times to indicate device startup without reset
*         LED blinks 5 times to indicate reset at device startup
* @param  void
* @retval MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT AppliNvm_FactorySettingReset(void)
{
  
  MOBLEUINT32 resetCount ;
  
    memcpy((void*)&resetCount,(void*)(APP_NVM_BASE + APP_NVM_RESERVED_SIZE), 8);
    AppliNvm_FlashErase(RESET_PAGE_NUMBER);   /** Flash memory 0x08080100**/
    resetCount = (resetCount << 1) ;
     /* Shifting for making Bits 0 from LSB as in STM32L4 initially bits are 1*/
    /* Check for "POWER_ON_OFF_CYCLES_FOR_UNPROVISIONING" times, that many bits shall be zero */
    if (resetCount > MASK_BITS_FOR_POWER_ON_OFF_CYCLES)
    {
#if !defined(DISABLE_TRACES)
      //printf("reset count before writing to memory 0x%.8x at address 0x%.8x\r\n", resetCount, APP_NVM_BASE + APP_NVM_RESERVED_SIZE);
#endif
      AppliNvm_FlashProgram((APP_NVM_BASE + APP_NVM_RESERVED_SIZE), (uint8_t*)&resetCount, 8);
  /*    memcpy((void*)&temp, (void*)(APP_NVM_BASE + APP_NVM_RESERVED_SIZE), 8);*/
      
      /* Blink twice to indicate device startup 
         1 second to blink once and 2 seconds to blink twice */
      for (MOBLEUINT8 i=0; i< ON_TIME_IN_SECONDS_FOR_POWER_CYCLING; i++)
      {
        Appli_LedBlink();
        STLBLE_PRINTF("111\r\n");
      }
      
      AppliNvm_FlashErase(RESET_PAGE_NUMBER);  /*erase the reset count*/
    }
    else /* Device is forced to factory reset, 5 LSBs are zero */
    {
      /* Unprovision node */
    	status_Mesh = BluenrgMesh_Unprovision();
      STLBLE_PRINTF("BlueMesh Status 3 BluenrgMesh_Unprovision:  %d \r\n", status_Mesh);
      

       AppliNvm_FlashErase(RESET_PAGE_NUMBER);
      
       status_Mesh = BluenrgMesh_SetUnprovisionedDevBeaconInterval(100);
       STLBLE_PRINTF("BlueMesh Status 4 BluenrgMesh_SetUnprovisionedDevBeaconInterval:  %d \r\n", status_Mesh);
      
      /* Blink 5 times to indicate factory setting reset */
      for (MOBLEUINT8 i=0; i< BLINK_TIMES_FOR_FACTORY_RESET ; i++)
      {
        Appli_LedBlink();
        STLBLE_PRINTF("unpro\r\n");
      }
      
      STLBLE_PRINTF("Reset to factory settings \r\n");
    }
  
  return MOBLE_RESULT_SUCCESS;
}

/**
* @brief  Save model states in nvm
* @param  model state buff
* @param  model state buff size
* @retval MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT AppliNvm_SaveModelState(uint8_t* state, uint16_t size)
{
  MOBLE_RESULT result = MOBLE_RESULT_FAIL; /* if save model state not defined, return MOBLE_RESULT_FAIL */
  
#if (SAVE_MODEL_STATE_NVM == 1)
  
  if (size > APP_NVM_MODEL_SIZE)
  {
    /* incorrect size */
    result = MOBLE_RESULT_FAIL;
  }
  else
  {
    AppliNvm_FlashErase(MODEL_PAGE_NUMBER);   /* Model data 0x08080800 */
//    printf("models data before writing to memory 0x%.8x at address 0x%.8x\r\n", *state, APP_NVM_BASE + APP_NVM_GENERIC_MODEL_OFFSET);

    result = AppliNvm_FlashProgram((APP_NVM_BASE + APP_NVM_GENERIC_MODEL_OFFSET), state, size);

  }
#endif /* SAVE_MODEL_STATE_NVM */  
  return result;
}


/**
* @brief  Load Model states from nvm
* @param  model state buff
* @param  model state buff size
* @retval MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT AppliNvm_LoadModelState(uint8_t state[], uint8_t* size)
{  
#if (SAVE_MODEL_STATE_NVM == 1)  
  MOBLE_RESULT result = MOBLE_RESULT_SUCCESS;
  memcpy((void*)state, (void*)(APP_NVM_BASE + APP_NVM_GENERIC_MODEL_OFFSET), APP_NVM_MODEL_SIZE);
 /*  printf("ReadLoadModel[%.8x]:", APP_NVM_BASE + APP_NVM_GENERIC_MODEL_OFFSET); */
/*   for (int i = 0; i < APP_NVM_MODEL_SIZE ; i++)
    {
      printf(" 0x%.8x", *((uint8_t*)(state+i)));
    }
    printf("\r\n"); */

  *size = APP_NVM_MODEL_SIZE;
  
  return result;  
#else /* SAVE_MODEL_STATE_NVM */  
  *size = 0;
  return MOBLE_RESULT_SUCCESS;
#endif /* SAVE_MODEL_STATE_NVM */

}

/**
* @brief  Process NVM erase and write requests
* @param  void
* @retval void
*/
void AppliNvm_Process(void)
{

}

/**
* @brief  Fuction used to set the flag which is responsible for storing the 
  states in flash.
* @param  void
* @retval void
*/
void AppliNvm_SaveMessageParam (void)
{
#ifdef SAVE_MODEL_STATE_FOR_ALL_MESSAGES
  PowerOnOff_flag = FLAG_SET;
#endif 
} 
