/**
  ******************************************************************************
  * @file    des_ecb.h
  * @author  MCD Application Team
  * @version V2.0.6
  * @date    25-June-2013
  * @brief   DES in ECB Mode
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
/*!
* \page Tutorial_DES_ECB DES ECB Tutorial
*
* Here follows an example of DES-ECB encryption.
* Please remember that before starting to call the Encryption function the context \b requires user
* initialization. The Flags member must be initialized prior to calling the
*  init function. Look at the each function documentation to see what is needed prior of calling.
*
* The API functions to be used are:
*  - \ref DES_ECB_Encrypt_Init initialize an \ref DESECBctx_stt context with key
*  - \ref DES_ECB_Encrypt_Append process the input and produces the output.
*    It can be called multiple times but the input size must be multiple of 8.
*  - \ref DES_ECB_Encrypt_Finish can be called only one time for the finalization process
*
* The following code performs a ECB encryption with DES of 1024 in 4 Append calls.
* \code
* #include <stdio.h>
* #include "crypto.h"
* int32_t main()
* {
*   uint8_t key[CRL_DES_KEY]={0,1,2,3,4,5,6,7};
*   uint8_t plaintext[1024]={...};
*   uint8_t ciphertext[1024];
*   // outSize is for output size, retval is for return value
*   int32_t outSize, retval;
*   DESECBctx_stt DESctx_st; // The DES context
*
*   // Initialize Context Flag with default value
*   DESctx_st.mFlags = E_SK_DEFAULT;
*   // call init function
*   retval = DES_ECB_Encrypt_Init(&DESctx_st, key, NULL);
*   if (retval != DES_SUCCESS)
*   {  ... }
*
*   // Loop to perform four calls to DES_CBC_Encrypt_Append, each processing 256 bytes
*   for (i = 0; i < 1024; i += 256)
*   { //Encrypt i bytes of plaintext. Put the output data in ciphertext and number
*     //of written bytes in outSize
*     retval = DES_ECB_Encrypt_Append(&DESctx_st, plaintext, 256, ciphertext, &outSize);
*     if (retval != DES_SUCCESS)
*     {  ... }
*   }
*   //Do the finalization call (in CBC it will not return any output)
*   retval = DES_ECB_Encrypt_Finish(&DESctx_st, ciphertext+outSize, &outSize );
*   if (retval != DES_SUCCESS)
*   {  ... }
* }
* \endcode
*/
#ifndef __CRL_DES_ECB_H__
#define __CRL_DES_ECB_H__

#ifdef __cplusplus
 extern "C" {
#endif

/** @ingroup DESECB
  * @{
  */
  
typedef DESCBCctx_stt DESECBctx_stt; /*!< Structure for the context of a DES operation */
  
#ifdef INCLUDE_ENCRYPTION
/* load the key and ivec, eventually performs key schedule, init hw, etc. *****/
int32_t DES_ECB_Encrypt_Init(DESECBctx_stt *P_pDESECBctx, const uint8_t *P_pKey, const uint8_t *P_pIv);

/* launch crypto operation , can be called several times **********************/
int32_t DES_ECB_Encrypt_Append (DESECBctx_stt *P_pDESECBctx,
                                const uint8_t *P_pInputBuffer,
                                int32_t        P_inputSize,
                                uint8_t       *P_pOutputBuffer,
                                int32_t       *P_pOutputSize);

/* Possible final output ******************************************************/
int32_t DES_ECB_Encrypt_Finish (DESECBctx_stt *P_pDESECBctx,
                                uint8_t       *P_pOutputBuffer,
                                int32_t       *P_pOutputSize);
#endif /* ECB Encryption */
#ifdef INCLUDE_DECRYPTION
/* load the key and ivec, eventually performs key schedule, init hw, etc. *****/
int32_t DES_ECB_Decrypt_Init (DESECBctx_stt *P_pDESECBctx, const uint8_t *P_pKey, const uint8_t *P_pIv);

/* launch crypto operation , can be called several times **********************/
int32_t DES_ECB_Decrypt_Append (DESECBctx_stt *P_pDESECBctx,
                                const uint8_t *P_pInputBuffer,
                                int32_t        P_inputSize,
                                uint8_t       *P_pOutputBuffer,
                                int32_t       *P_pOutputSize);

/* Possible final output ******************************************************/
int32_t DES_ECB_Decrypt_Finish (DESECBctx_stt *P_pDESECBctx,
                                uint8_t       *P_pOutputBuffer,
                                int32_t       *P_pOutputSize);
#endif /* ECB Decryption */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __CRL_DES_ECB_H__*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
