/**
******************************************************************************
* @file    bluenrg_mesh.h
* @author  BLE Mesh Team
* @version V1.3.0
* @date    19-09-2019
* @brief   Header file for the BlueNRG-Mesh stack 
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BLUENRG_MESH_
#define _BLUENRG_MESH_

#include "types.h"
#include "hal_types.h"
#define BLUENRG_MESH_APPLICATION_VERSION "1.3.0"
/**
* \mainpage ST BlueNRG-Mesh Solutions Bluetooth LE Mesh Library
*
* \version 1.11.000
*
* \subsection contents_sec Contents
*
* -# \ref overview_sec
* -# \ref supported_devices
* -# \ref install_sec
* -# \ref brief_descr_sec
* -# \ref other_info_sec
* -# \link bluenrg_mesh.h ST BlueNRG-Mesh Library User API \endlink
* -# \link types.h ST BlueNRG-Mesh Library Data Types \endlink
*
* \subsection overview_sec Overview
* 1) Overview: 
* BlueNRG-Mesh is a solution for connecting multiple BLE (Bluetooth Low Energy) 
* devices in Mesh networking for IoT (Internet of Things) solutions. 
* It enables the Bluetooth enabled devices into a powerful, integrated, 
* range-extending Mesh network with true two-way communication.
* The solution contains the core functionality required to form the secure 
* communication network and provide flexibility to the developer to develop applications.
* The solution is available for BlueNRG product family. 
*
* \subsection supported_devices Supported devices
* 2) Supported devices:
* The solution is available for BlueNRG product family:
* - BlueNRG-2
* - BlueNRG-1
*
* \subsection install_sec Installation
* 3) Installation:
* To use ST BlueNRG-Mesh Library in any application the following should be done:
* - \a libBlueNRG_Mesh_CM0.a file should be linked to the application 
*                                     for BlueNRG-1, BlueNRG-2 for IAR or Keil Compiler
*
*
* Proper operation of ST BlueNRG-Mesh Library requires:
* - Call BluenrgMesh_Init() function before any calls to library functions
* - Call BluenrgMesh_Process() on each iteraction in application main loop
*
* \subsection brief_descr_sec API brief description
* 4) API brief description:
* ST BlueNRG-Mesh Library sends and receives data from remote devices by itself when 
* required and provides data to the user applicaton by calling the appropriate callbacks.
*
* User application operation:
* - User application defines a callback map (\a MOBLE_VENDOR_CB_MAP)
* - The callback map is provided to ST BlueNRG-Mesh Library in BluenrgMesh_SetVendorCbMap() 
*                                                          function
* - The callbacks from the map are invoked by the ST BlueNRG-Mesh Library upon data 
*                                                          or request receival.
*
* \subsection other_info_sec Licensing and other Information
* 5) Licensing and other Information:
* BlueNRG-Mesh is built over Motorola's Mesh over Bluetooth Low Energy Technology (MoBLE)
* ST has done suitable updates required for application and networking features
*/
/**
* \file bluenrg_mesh.h
* \brief This file defines ST BlueNRG-Mesh Solutions Bluetooth LE Mesh Library user API.
*
* This file defines ST BlueNRG-Mesh Solutions Bluetooth LE Mesh Library user API. 
* Please refer to the desript
*/
#include <stdint.h>

/**
* \brief Output OOB Action values (provisioner)
*/
#define OUTPUT_OOB_ACTION_BIT_BLINK               (1 << 0) /**< Blink */
#define OUTPUT_OOB_ACTION_BIT_BEEP                (1 << 1) /**< Beep */
#define OUTPUT_OOB_ACTION_BIT_VIBRATE             (1 << 2) /**< Vibrate */
#define OUTPUT_OOB_ACTION_BIT_DISPLAY_NUM         (1 << 3) /**< Display Numeric */

/**
* \brief Input OOB Action values (unprovisioned node)
*/
#define INPUT_OOB_ACTION_BIT_PUSH                 (1 << 0) /**< Push */
#define INPUT_OOB_ACTION_BIT_TWIST                (1 << 1) /**< Twist */
#define INPUT_OOB_ACTION_BIT_ENTER_NUM            (1 << 2) /**< Enter Number */

/** \brief List of status values for responses. */
typedef enum _MOBLE_COMMAND_STATUS
{
  /** \brief Successful response
  * Returned when the packet is successively processed.
  */
  STATUS_SUCCESS = 0x00,
  
  /** \brief Invalid command response
  * Returned when the command in the packet is not supported.
  */
  STATUS_INVALID_COMMAND = 0x01,
  
  /** \brief Invalid address response
  * Returned when an address of a data element in the packet is not supported.
  */
  STATUS_INVALID_ADDRESS = 0x02,
  
  /** \brief Invalid data response
  * Returned when the data in the packet is invalid.
  */
  STATUS_INVALID_DATA = 0x03,
  
  /** \brief Device failure response
  * Returned when the device is unable to process packet.
  */
  STATUS_DEVICE_ERROR = 0x04
    
} MOBLE_COMMAND_STATUS;

/**
* This structure contains transmit receive parameters
*/ 
typedef struct
{
    uint16_t gapBetweenPktTransmission;/* Gap between successive packet transmission */
} tr_params_t;

/**
* This structure contains Low Power feature initialization parameters
*/ 
typedef struct
{
    uint8_t rssiFactor;
    uint8_t receiveWindowFactor;
    uint8_t minQueueSizeLog;
    uint8_t receiveDelay;
    uint32_t pollTimeout;
    uint8_t friendRequestFrequency;
    uint32_t friendPollFrequency;
    uint8_t receiveWindow;
    uint8_t subscriptionList;
    int8_t minRssi;
    uint8_t noOfRetry;
} lpn_params_t;

/**
* This structure contains Friend feature initialization parameters
*/ 
typedef struct
{   
    uint8_t noOfLpn;
} fn_params_t;

/**
* This structure contains static OOB parameters
*/ 
typedef struct
{
    uint8_t pubKeyTypeOob;  /* Used Public Key: OOB / No OOB */   
    const uint8_t *pubKey;    /* Pointer to array containing Public Key of the device */
    const uint8_t *privKey;   /* Pointer to array containing Private Key of the device*/ 
    uint8_t staticOobSize;    /* Size of Static OOB array */
    const uint8_t *staticOob; /* Pointer to array containing Static OOB info of the device */
    uint8_t OutputOobSize;    /* Size of Output OOB value */
    uint8_t OutputOobAction;  /* Size of Output OOB Action */
    void (*OutputOOBAuthCb)(MOBLEUINT8* oobData, MOBLEUINT8 size); /* Callback to Output OOB data */
    uint8_t InputOobSize;    /* Size of Input OOB value */
    uint8_t InputOobAction;  /* Size of Input OOB Action */
    MOBLEUINT8* (*InputOOBAuthCb)(MOBLEUINT8 size); /* Callback to Input OOB data */
} prvn_params_t;

/**
* Structure contains neighbor table initialization parameters
*/ 
typedef struct
{
    uint8_t count;
    uint8_t aliveTime;
    uint8_t unprvnd_dev_beacon_ntu;
    uint8_t secure_net_beacon_ntu;
    uint8_t msg_ttlx_ntu;
} neighbor_table_init_params_t;

/**
* Structure contains neighbor parameters
*/
typedef struct
{
  MOBLEUINT8 bdAddr[6];
  MOBLEBOOL   provisioned; 
  MOBLEUINT8 uuid[16];
  MOBLE_ADDRESS networkAddress;
  MOBLEUINT8 rssi;
} neighbor_params_t;

/** \brief Callback map */
typedef struct
{
  /** \brief Write local data callback.
  * Called when the device gets a request to modify its data. Such a request is 
  * made via a call to \a BluenrgMesh_SetRemotePublication
  * on a remote device.
  * User is responsible for deserializing the data.
  * \param[in] peer Source network address.
  * \param[in] dst_peer Destination address set by peer.
  * \param[in] offset Address of data in the data map.
  * \param[in] data Data buffer. Contains vendor-specific representation of data.
  * \param[in] length Data buffer length in bytes.
  * \param[in] response Flag if response is required.
  * \return MOBLE_RESULT_SUCCESS on success.
  */
  MOBLE_RESULT (*WriteLocalData)(MOBLE_ADDRESS peer, MOBLE_ADDRESS dst_peer, 
                                 MOBLEUINT8 offset, MOBLEUINT8 const *data, 
                                 MOBLEUINT32 length, MOBLEBOOL response);
  
  /** \brief Read local data callback.
  * Called when the device gets a request to provide its data. Such a request is 
  *         made via a call to \a _ReadRemoteData on a remote device.
  * User is responsible for serializing the data. After this callback 
  *              successfully returns, data is sent back to the requesting peer.
  * \param[in] peer Source network address.
  * \param[in] dst_peer Destination address set by peer.
  * \param[in] offset Address of data in the data map.
  * \param[in] response Flag if response is required.
  * \return MOBLE_RESULT_SUCCESS on success.
  */
  MOBLE_RESULT (*ReadLocalData)(MOBLE_ADDRESS peer, MOBLE_ADDRESS dst_peer, 
                                MOBLEUINT8 offset, MOBLEUINT8 const *data, 
                                MOBLEUINT32 length, MOBLEBOOL response);
  
  /** \brief On Response data callback.
  * Called when the device gets a request to provide its data. Such a request is 
  *         made via a call to \a Send response on a remote device.
  * User is responsible for serializing the data. After this callback 
  *              successfully returns, data is sent back to the requesting peer.
  * \param[in] peer Source network address.
  * \param[in] dst_peer Destination address set by peer.
  * \param[in] offset Address of data in the data map.
  * \param[in] response Flag if response is required.
  * \return MOBLE_RESULT_SUCCESS on success.
  */                                 
  MOBLE_RESULT (*OnResponseData)(MOBLE_ADDRESS peer_addr, MOBLE_ADDRESS dst_peer,
                                 MOBLEUINT8 command, MOBLEUINT8 const *pRxData, 
                                 MOBLEUINT32 dataLength, MOBLEBOOL response);

} MOBLE_VENDOR_CB_MAP;

/** \brief Hardware function Callback map */
typedef struct 
{ 
  /* Structure for setting the hardware configuration by the user */
  MOBLE_RESULT (*BLE_Stack_Initialization)(void);
  
  /* Structure for setting the Tx Power by the user */
  MOBLE_RESULT (*BLE_Set_Tx_Power)(void);
  
  /*This event indicates that a new connection has been created.*/
  void (*GATT_Connection_Complete)(void);
  
  /*This event occurs when a connection is terminated*/
  void (*GATT_Disconnection_Complete)(void);
  
  /*This event occurs for a Unprovisioned Node Identification*/
  void (*Unprovision_Identify_Cb)(MOBLEUINT8 data);
  
  /* Call back function for setting UUID value by the user  
     when BluenrgMesh_Init() function called*/
  MOBLE_RESULT (*Set_Device_UUID) (MOBLEUINT8 *data); 
  
  /* Call back function for setting CID and PID by the user */ 
  MOBLE_RESULT (*Set_Product_Info) (MOBLEUINT8 *data); 
  
  /*This event sets the number of elements in a Node*/
  MOBLEUINT8 (*Number_Of_Capabilities_Element)(void);
  
  /*This event disables network layer filter to sniff all the packets*/
  MOBLEUINT8 (*Disable_Filter)(void);

} MOBLE_USER_BLE_CB_MAP;


/** \brief User Application function Callback map */
typedef struct 
{
  /* Call back function to switch on or off the LED*/
  MOBLE_RESULT (*LedStateCtrl)(MOBLEUINT16 control);  
  
} MOBLE_USER_INTF_CB_MAP;

/** \brief of opcode table structure.
  * This is the structure of opcode of set, get and status message with the maximum 
  *              and maximum parameter value for set,get and status messages.
  * \param[in] opcode: opcode of message type.
  * \param[in] reliable :wheather mesaage is acknowledge or unacknowledge. 
  * \param[in] min_payload_size: minimum length of message.
  * \param[in] max_payload_size: maximum length of message.
  * \param[in] response_opcode: opcode for status message
  * \param[in] min_response_size: minimum length of status message.
  * \param[in] max_response_size: maximum length of status message.
 **/
typedef struct 
{
    MOBLEUINT32 opcode;
    MOBLEBOOL reliable;
    MOBLEUINT16 min_payload_size;
    MOBLEUINT16 max_payload_size;
    MOBLEUINT16 response_opcode;
    MOBLEUINT16 min_response_size;
    MOBLEUINT16 max_response_size;    
} MODEL_OpcodeTableParam_t;

/** \brief Callback map */
typedef struct
{  
  /** \brief Get opcode table callback.
  * This function is a callback to get status opcode form the received opcode 
  * from client side.
  * \param[in] Pointer to get the opcode table address
  * \param[in] length of the opcode.
  * \return MOBLE_RESULT_SUCCESS on success.
  */
   
  MOBLE_RESULT (*ModelSIG_GetOpcodeTableCb)(const MODEL_OpcodeTableParam_t **data, 
                                    MOBLEUINT16 *length);
  
  /** \brief get message/status message process callback
  * This function called when there will acknowleged message received or Get message is
  * is received to get the status of the message.
  * \param[in] peer Source network address.
  * \param[in] dst_peer Destination address set by peer.
  * \param[in] opcode to be processed
  * \param[in] data Data buffer. to be sent back in status
  * \param[in] length Data buffer length in bytes.
  * \param[in] response Flag if response is required.
  * \return MOBLE_RESULT_SUCCESS on success.
  */ 
  MOBLE_RESULT (*ModelSIG_GetRequestCb)(MOBLE_ADDRESS peer_addr, 
                                    MOBLE_ADDRESS dst_peer, 
                                    MOBLEUINT16 opcode, 
                                    MOBLEUINT8 *data, 
                                    MOBLEUINT32 *res_length,
                                    MOBLEUINT8 const *pData,
                                    MOBLEUINT32 length,
                                    MOBLEBOOL response); 
  
  /** \brief set message process callback
  * This function called when there will set message is received.
  * \param[in] peer Source network address.
  * \param[in] dst_peer Destination address set by peer.
  * \param[in] opcode to be processed
  * \param[in] data Data buffer. to be sent back in status
  * \param[in] length Data buffer length in bytes.
  * \param[in] response Flag if response is required.
  * \return MOBLE_RESULT_SUCCESS on success.
  */ 
  MOBLE_RESULT (*ModelSIG_SetRequestCb)(MOBLE_ADDRESS peer_addr, 
                                    MOBLE_ADDRESS dst_peer, 
                                    MOBLEUINT16 opcode, 
                                    MOBLEUINT8 const *data, 
                                    MOBLEUINT32 length, 
                                    MOBLEBOOL response);

} MODEL_SIG_cb_t;


/** \brief Callback map */
typedef struct
{  
  /** \brief Get opcode table callback.
  * This function is a callback to get status opcode form the received opcode 
  * from client side.
  * \param[in] Pointer to get the opcode table address
  * \param[in] length of the opcode.
  * \return MOBLE_RESULT_SUCCESS on success.
  */
   
  MOBLE_RESULT (*ModelVendor_GetOpcodeTableCb)(const MODEL_OpcodeTableParam_t **data, 
                                    MOBLEUINT16 *length);
  
  /** \brief get message/status message process callback
  * This function called when there will acknowleged message received or Get message is
  * is received to get the status of the message.
  * \param[in] peer Source network address.
  * \param[in] dst_peer Destination address set by peer.
  * \param[in] opcode to be processed
  * \param[in] data Data buffer. to be sent back in status
  * \param[in] length Data buffer length in bytes.
  * \param[in] response Flag if response is required.
  * \return MOBLE_RESULT_SUCCESS on success.
  */ 
  MOBLE_RESULT (*ModelVendor_GetRequestCb)(MOBLE_ADDRESS peer_addr, 
                                    MOBLE_ADDRESS dst_peer, 
                                    MOBLEUINT16 opcode, 
                                    MOBLEUINT8 *data, 
                                    MOBLEUINT32 *res_length,
                                    MOBLEUINT8 const *pData,
                                    MOBLEUINT32 length,
                                    MOBLEBOOL response); 
  
  /** \brief set message process callback
  * This function called when there will set message is received.
  * \param[in] peer Source network address.
  * \param[in] dst_peer Destination address set by peer.
  * \param[in] opcode to be processed
  * \param[in] data Data buffer. to be sent back in status
  * \param[in] length Data buffer length in bytes.
  * \param[in] response Flag if response is required.
  * \return MOBLE_RESULT_SUCCESS on success.
  */ 
  MOBLE_RESULT (*ModelVendor_SetRequestCb)(MOBLE_ADDRESS peer_addr, 
                                    MOBLE_ADDRESS dst_peer, 
                                    MOBLEUINT16 opcode, 
                                    MOBLEUINT8 const *data, 
                                    MOBLEUINT32 length, 
                                    MOBLEBOOL response);

} MODEL_Vendor_cb_t;

typedef struct
{
  MOBLEUINT8* pbuff_dyn ;
  const uint16_t dyn_buff_size;
  const uint16_t friend_lp_buff_size;
  const uint16_t max_appli_pkt_size;
  const uint16_t neighbor_table_buff_size;
} DynBufferParam_t;


typedef struct
{
  MOBLEUINT8* pbdaddr ;
  const tr_params_t* pTrParams;
  const fn_params_t* pFnParams;
  const lpn_params_t* pLpnParams;
  const neighbor_table_init_params_t* pNeighborTableParams;
  const uint16_t features;
  const uint8_t prvnBearer;
  const prvn_params_t* pPrvnParams;
  const DynBufferParam_t* pDynBufferParam;
} Mesh_Initialization_t;

/******************************************************************************/
/*                         BlueNRG-Mesh stack functions                       */
/******************************************************************************/

/** \brief ST BlueNRG-Mesh Library initialization
*
* This function should be called to initialize ST BlueNRG-Mesh Library.
* Other ST BlueNRG-Mesh Library functions should NOT be called until the library is initialized
*
* \param[in] bdaddr A pointer to the array with MAC address. If equals NULL then 
*                          default MAC adress is used, i.e. it is not hanged.
* \param[in] features Features to be supported by library
*                     Bit0 Relay feature
*                     Bit1 Proxy feature
*                     Bit2 Friend feature
*                     Bit3 Low power feature
* \param[in] LpnParams Init values corresponding to low power node performance
* \return MOBLE_RESULT_SUCCESS on success.
*
*/
MOBLE_RESULT BluenrgMesh_Init(const Mesh_Initialization_t* pInit_params);

/** \brief ST BlueNRG-Mesh Library Version
*
* This function can be called to get the latest library version.
* \return Pointer to string.
*
*/
char* BluenrgMesh_GetLibraryVersion(void);

/** \brief ST BlueNRG-Mesh Library Sub Version
*
* This function can be called to get the latest library sub version.
* \return Pointer to string.
*
*/
char* BluenrgMesh_GetLibrarySubVersion(void);

/** \brief ST BlueNRG-Mesh Library main task processing function
*
* This function should be called in user application main loop.
* \return MOBLE_RESULT_SUCCESS on success.
*
*/
MOBLE_RESULT BluenrgMesh_Process(void);

/* brief set Generic model
*\returnMOBLE_RESULT_SUCCESS
*/

/** \brief Set callback map.
* \param[in] map callback map. If NULL, nothing is done.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetVendorCbMap(MOBLE_VENDOR_CB_MAP const * map);

/** \brief Set remote data on the given peer. The usage of this API is depracated and replaced with
*                                                     BluenrgMesh_SetRemotePublication
* User is responsible for serializing data into \a data buffer. Vendor_WriteLocalDataCb 
*                                  callback will be called on the remote device.
* \param[in] peer Destination address. May be set to MOBLE_ADDRESS_ALL_NODES to broadcast data.
* \param[in] elementIndex index of the element
* \param[in] command vendor model commands 
* \param[in] data Data buffer.
* \param[in] length Length of data in bytes.
* \param[in] response If 'MOBLE_TRUE', used to get the response. If 'MOBLE_FALSE', no response 
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetRemoteData(MOBLE_ADDRESS peer, MOBLEUINT8 elementIndex,
                                       MOBLEUINT16 command, MOBLEUINT8 const * data, 
                                       MOBLEUINT32 length, MOBLEBOOL response, 
                                       MOBLEUINT8 isVendor);


/** \brief Set remote publication for the given Model ID & node Address
* User is responsible for serializing data into \a data buffer. Vendor_WriteLocalDataCb 
*                                  callback will be called on the remote device.
* \param[in] modelId ID of the model. 
* \param[in] srcAddress element Address of the Node
* \param[in] command vendor model commands 
* \param[in] data Data buffer.
* \param[in] length Length of data in bytes.
* \param[in] response If 'MOBLE_TRUE', used to get the response. If 'MOBLE_FALSE', no response 
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetRemotePublication(MOBLEUINT32 modelId, MOBLE_ADDRESS srcAddress,
                                              MOBLEUINT16 command, MOBLEUINT8 const * data, 
                                              MOBLEUINT32 length, MOBLEBOOL response,
                                              MOBLEUINT8 isVendor);

/** \brief Vendor Model Set remote data on the given peer.
* User is responsible for serializing data into a data buffer. 
* \param[in] peer Destination address. May be set to MOBLE_ADDRESS_ALL_NODES to broadcast data.
* \param[in] src_addr index of the element which is generating the data
* \param[in] command vendor model commands 
* \param[in] data Data buffer.
* \param[in] length Length of data in bytes.
* \param[in] response If 'MOBLE_TRUE', used to get the response. If 'MOBLE_FALSE', no response 
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT Vendor_WriteRemoteData (MOBLEUINT32 vendorModelId,
                                     MOBLE_ADDRESS src_addr, 
                                     MOBLE_ADDRESS dst_peer, 
                                     MOBLEUINT8 command, 
                                     MOBLEUINT8 const *data, 
                                     MOBLEUINT32 length, 
                                     MOBLEBOOL response);
  
/** \brief Read remote data on the given peer.
* User is responsible for serializing data into \a data buffer. Vendor_ReadLocalDataCb 
*                                  callback will be called on the remote device.
*                                  It is reliable command
* \param[in] peer Destination address. May be set to MOBLE_ADDRESS_ALL_NODES to broadcast data.
* \param[in] elementIndex index of the element
* \param[in] command vendor model commands 
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_ReadRemoteData(MOBLE_ADDRESS peer,
                                        MOBLEUINT8 elementIndex, 
                                        MOBLEUINT16 command,
                                        MOBLEUINT8 const * data, 
                                        MOBLEUINT32 length);

/** \brief Send response on received packet.The usage of this API is depracated and replaced with VendorModel_SendResponse
* \param[in] peer Destination address. Must be a device address (0b0xxx xxxx xxxx xxxx, but not 0).
* \param[in] dst Source Address of Node
* \param[in] status Status of response.
* \param[in] data Data buffer.
* \param[in] length Length of data in bytes. Maximum accepted length is 8. 
*             If length is zero, no associated data is sent with the report.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SendResponse(MOBLE_ADDRESS peer, MOBLE_ADDRESS dst, 
                                      MOBLEUINT8 status, MOBLEUINT8 const * data, 
                                      MOBLEUINT32 length);

/** \brief Send response on received packet.
* \param[in] peer Destination address. Must be a device address (0b0xxx xxxx xxxx xxxx, but not 0).
* \param[in] dst Source Address of Node
* \param[in] status Status of response.
* \param[in] data Data buffer.
* \param[in] length Length of data in bytes. Maximum accepted length is 8. 
*             If length is zero, no associated data is sent with the report.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT VendorModel_SendResponse(MOBLEUINT16 vendorModelId, MOBLE_ADDRESS peer, MOBLE_ADDRESS dst, 
                                      MOBLEUINT8 status, MOBLEUINT8 const * data, 
                                      MOBLEUINT32 length);

/** \brief Sensor Send response on received packet.
* \param[in] peer Destination address. Must be a device address (0b0xxx xxxx xxxx xxxx, but not 0).
* \param[in] data Data buffer.
* \param[in] length Length of data in bytes. Maximum accepted length is 8. 
*             If length is zero, no associated data is sent with the report.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT Model_SendResponse(MOBLE_ADDRESS src_peer,MOBLE_ADDRESS dst_peer ,
                                              MOBLEUINT16 opcode,MOBLEUINT8 const *pData,MOBLEUINT32 length); 

/** \brief initialize unprovisioned node to be provisioned.
* \param None
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_InitUnprovisionedNode(void);

/** \brief Enable provision procedure for provisioned node.
* \param None.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_InitProvisionedNode(void);

/** \brief Check if node configures as Unprovisioned node.
* \return MOBLE_TRUE if node configured as Unprovisioned node. MOBLE_FALSE otherwise.
*/
MOBLEBOOL BluenrgMesh_IsUnprovisioned(void);

/** \brief Unprovisions the node if it is provisioned.
* \return MOBLE_RESULT_SUCCESS on success, MOBLE_RESULT_FALSE if node us already 
*                               unprovisioned, and failure code in other cases.
*/
MOBLE_RESULT BluenrgMesh_Unprovision(void);

/** \brief Set BLE Hardware init callback
* \param _cb callback
* \return MOBLE_RESULT_SUCCESS on success, MOBLE_RESULT_FAIL if failure code.
*/
MOBLE_RESULT BluenrgMesh_BleHardwareInitCallBack(MOBLE_USER_BLE_CB_MAP const * _cb);

/** \brief Get provisioning process state
* \return 0,1,2,3,4,5,6 during provisioning, else 7.
*/
MOBLEUINT8 BluenrgMesh_GetUnprovisionState(void);

/** \brief Get mesh address of a node
*
* This function gets address of a node. If node is unprovisioned then 
*                                         MOBLE_ADDRESS_UNASSIGNED is returned.
*
* \return mesh address of a node.
*
*/
MOBLE_ADDRESS BluenrgMesh_GetAddress(void);

/** \brief Get Publish address of a node
*
* This function gets address of a node. 
*
* \return mesh address of a node.
*
*/
MOBLE_ADDRESS BluenrgMesh_GetPublishAddress(MOBLEUINT8 elementNumber, MOBLEUINT32 modelId);

/** \brief Get Subscription address of a node
*
* This function gets addresses of selected element. 
*
* \return MOBLE_RESULT_SUCCESS on success, MOBLE_RESULT_FAIL if failure code
*
*/
MOBLE_RESULT BluenrgMesh_GetSubscriptionAddress(MOBLE_ADDRESS *addressList, 
                                                     MOBLEUINT8 *sizeOfList, 
                                                     MOBLEUINT8 elementNumber,
                                                     MOBLEUINT32 modelId);

/** \brief Set default TTL value.
* When message is sent to mesh network, it contains TTL field. User shall call 
* this function to set TTL value used during message transmission.
* \param[in] ttl TTL value. Supported values are 0-127.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetTTL(MOBLEUINT8 ttl);

/** \brief Get default TTL value.
* \return Default TTL value.
*/
MOBLEUINT8 BluenrgMesh_GetTTL(void);

/** \brief Set Netwrok Transmit Count value.
* When message is sent to mesh network, it is replicated NetworkTransmitCount 
* + 1 times. User shall call this function to set Netwrok Transmit value used 
* during message transmission.
* \param[in] count Network Transmit value. Supported values are 1-8.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetNetworkTransmitCount(MOBLEUINT8 count);

/** \brief Get Netwrok Transmit Count value.
* \return Default Network Transmit Count value.
*/
MOBLEUINT8 BluenrgMesh_GetNetworkTransmitCount(void);

/** \brief Set Relay Retransmit Count value.
* When message is relayed by mesh network relay, it is replicated 
* RelayRetransmitCount + 1 times. User shall call this function to set Relay 
* Retransmit value used during message transmission.
* \param[in] count Relay Retransmit value. Supported values are 1-8.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetRelayRetransmitCount(MOBLEUINT8 count);

/** \brief Get Relay Retransmit Count value.
* \return Default Relay retransmit Count value.
*/
MOBLEUINT8 BluenrgMesh_GetRelayRetransmitCount(void);

/** \brief Enable or disable relay feature. Feature can be changed only if it is supported 
*          0 - disable, 1 - enable
* \return MOBLE_RESULT_FALSE if no change occur
*         MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT BluenrgMesh_SetRelayFeatureState(MOBLEUINT8 state);

/** \brief Enable or disable proxy feature. Feature can be changed only if it is supported 
*          0 - disable, 1 - enable
* \return MOBLE_RESULT_FALSE if no change occur
*         MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT BluenrgMesh_SetProxyFeatureState(MOBLEUINT8 state);

/** \brief Enable or disable friend feature. Feature can be changed only if it is supported 
*          0 - disable, 1 - enable
* \return MOBLE_RESULT_FALSE if no change occur
*         MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT BluenrgMesh_SetFriendFeatureState(MOBLEUINT8 state);

/** \brief Disable low power feature only if it is supported and enabled
*          0 - disable, low power feature can't be enabled using BluenrgMesh_SetLowPowerFeatureState
* \return MOBLE_RESULT_FALSE if no change occur
*         MOBLE_RESULT_SUCCESS on success
*/
MOBLE_RESULT BluenrgMesh_SetLowPowerFeatureState(MOBLEUINT8 state);

/** \brief Get features state
*          Bit0: Relay feature. 0 - disabled, 1 - enabled
*          Bit1: Proxy feature. 0 - disabled, 1 - enabled
*          Bit2: Friend feature. 0 - disabled, 1 - enabled
*          Bit3: Low Power feature. 0 - disabled, 1 - enabled
* \return Features state
*/
MOBLEUINT16 BluenrgMesh_GetFeatures(void);

/** \brief Set callback for handling heartbeat messages.
*
* \param[in] cb Callback
* \return MOBLE_RESULT_SUCCESS on success.
*
*/
MOBLE_RESULT BluenrgMesh_SetHeartbeatCallback(MOBLE_HEARTBEAT_CB cb);

/** \brief Set callback for attention timer.
* To be used for attention during provisioning and for health model
* For devices who want to implement actions corresponding to attention timer, set callback else do not set callback
* \param[in] cb Callback
* \return MOBLE_RESULT_SUCCESS on success.
*
*/
MOBLE_RESULT BluenrgMesh_SetAttentionTimerCallback(MOBLE_ATTENTION_TIMER_CB cb);

/** \brief Unprovision callback
* Callback on unprovision by provisioner
* \param[in] Unprovisioning reason
*
*/
void BluenrgMesh_UnprovisionCallback(MOBLEUINT8 reason);

/** \brief Provision callback
* Callback on Provision by provisioner
*
*/
void BluenrgMesh_ProvisionCallback(void);

/** \brief Set SIG Model callback map.
* \param[in] map callback map. If NULL, nothing is done.
* \count[in] count of the number of models defined in Application
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT  BluenrgMesh_SetSIGModelsCbMap(const MODEL_SIG_cb_t* pSig_cb, MOBLEUINT32 count);

/** \brief GetApplicationVendorModels
* \param[in] map callback map. If NULL, nothing is done.
* \count[in] count of the number of models defined in Application
* \return MOBLE_RESULT_SUCCESS on success.
*/
void GetApplicationVendorModels(const MODEL_Vendor_cb_t** pModelsTable, MOBLEUINT32* VendorModelscount);

/** \brief Returns sleep duration.
* going to sleep (or no call to BluenrgMesh_Process()) for this duration does not affect operation of mesh library
* \return Sleep duration.
*/
MOBLEUINT32 BluenrgMesh_GetSleepDuration(void);

/** \brief Upper tester data process.
* \param[in] testFunctionIndex: Test function indexes
* \param[in] testFunctionParm: Test function parameters
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_UpperTesterDataProcess(MOBLEUINT8 testFunctionIndex, MOBLEUINT8* testFunctionParm);

/** \brief String print callback function.
* \param[in] message: To be printed string pointer 
* \return void.
*/
void BluenrgMesh_PrintStringCb(const char *message);
/** \brief Data print callback function.
* \param[in] data: To be printed data pointer 
* \param[in] size: Size of data to be printed data pointer 
* \return void.
*/
void BluenrgMesh_PrintDataCb(MOBLEUINT8* data, MOBLEUINT16 size);


/** \brief Friend node callback corresponding to friendship established with low power node.
* \param[out] address of corresponding low power node.
* \param[out] receive delay of low power node (unit ms).
* \param[out] poll timeout of low power node (unit 100ms).
* \param[out] number of elements of low power node.
* \param[out] previous friend address of low power node (can be invalid address).
*/
void BluenrgMesh_FnFriendshipEstablishedCallback(MOBLE_ADDRESS lpnAddress,
                                                 MOBLEUINT8 lpnReceiveDelay,
                                                 MOBLEUINT32 lpnPollTimeout,
                                                 MOBLEUINT8 lpnNumElements,
                                                 MOBLE_ADDRESS lpnPrevFriendAddress);

/** \brief Friend node callback corresponding to friendship cleared with low power node.
* \param[out] reason of friendship clear.
*             0: reserved
*             1: friend request received from existing low power node (friend)
*             2: low power node poll timeout occurred
*             3: friend clear received
* \param[out] address of corresponding low power node.
*/
void BluenrgMesh_FnFriendshipClearedCallback(MOBLEUINT8 reason, MOBLE_ADDRESS lpnAddress);

/** \brief Low Power node callback corresponding to friendship established with friend node.
* \param[out] address of corresponding friend node.
*/
void BluenrgMesh_LpnFriendshipEstablishedCallback(MOBLE_ADDRESS fnAddress);

/** \brief Low Power node callback corresponding to friendship cleared with friend node.
* \param[out] reason of friendship clear.
*             0: reserved
*             1: No response received from friend node
* \param[out] address of corresponding friend node.
*/
void BluenrgMesh_LpnFriendshipClearedCallback(MOBLEUINT8 reason, MOBLE_ADDRESS fnAddress);

/** \brief To synchronize flash erase with sufficient available time w.r.t. next connection event.
* \return MOBLE_TRUE if no connection exists or sufficient time is available for flash erase operation.
*/
MOBLEBOOL BluenrgMesh_IsFlashReadyToErase(void);

/** \brief Stop ongoing scan (if in scan mode) and advertisement (if in adv mode).
*/
void BluenrgMesh_StopAdvScan(void);

/** \brief Set adv interval of provisioning service, 0 value results in stop.
*          Default value: 1000 ms
* \param[in] adv interval (ms), min interval value is 100 ms
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetProvisioningServAdvInterval(MOBLEUINT16 interval);

/** \brief Set interval of unprovisioned device beacon, 0 value results in stop.
*          Default value: 1000 ms
* \param[in] interval (ms) of beacons, min interval value is 100 ms
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetUnprovisionedDevBeaconInterval(MOBLEUINT16 interval);

/** \brief Set adv interval of proxy service.
*          Default value: 1000 ms
* \param[in] adv interval (ms), min interval value is 1000 ms
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetProxyServAdvInterval(MOBLEUINT16 interval);

/** \brief Set interval of secure network beacon.
*          Default value: 10000 ms
* \param[in] interval (ms) of beacons, min interval value is 10000 ms
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetSecureBeaconInterval(MOBLEUINT16 interval);

/** \brief Set interval of custom beacon, 0 value results in stop.
* \param[in] interval (ms) of beacons, min interval value is 1000 ms
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetCustomBeaconInterval(MOBLEUINT16 interval);

/** \brief Set custom beacon data.
*          If size > 31 bytes, beacon is rejected
* \param[out] beacon data buffer
* \param[out] size of beacon data
*/
void BluenrgMesh_CustomBeaconGeneratorCallback(void* buffer, int* size);

/** 
* @brief ApplicationGetSigModelList: This function provides the list of the 
*           SIG Models to the calling function
* @param pModels_sig_ID: Pointer of the array to be filled with SIG Models list
* @param elementIndex: Index of the element for Model List
* retval Count of the SIG Model Servers enabled in the Application
*/
MOBLEUINT8 ApplicationGetSigModelList(MOBLEUINT16* pModels_sig_ID, \
                                                        MOBLEUINT8 elementIndex);

/** 
* @brief ApplicationGetVendorModelList: This function provides the list of the 
*           Vendor Models to the calling function
* @param pModels_sig_ID: Pointer of the array to be filled with Vendor Models list
* @param elementIndex: Index of the element for Model List
* retval Count of the Vendor Model Servers enabled in the Application
*/
MOBLEUINT8 ApplicationGetVendorModelList(MOBLEUINT32* pModels_vendor_ID, \
                                                       MOBLEUINT8 elementIndex);

/** 
* @brief ApplicationChkSigModelActive: This function checks if a specific 
*          Model Server is active in the Model Server list
* @param modelID: Model Server ID received for the checking function
* retval Bool: True or False, if the Server ID matches with the list 
*/
MOBLEBOOL ApplicationChkSigModelActive(MOBLEUINT16 modelID);

/** 
* @brief ApplicationChkVendorModelActive: This function checks if a specific 
*          Model Server is active in the Vendor Model Server list
* @param modelID: Model Server ID received for the checking function
* retval Bool: True or False, if the Server ID matches with the list 
*/
MOBLEBOOL ApplicationChkVendorModelActive(MOBLEUINT32 modelID);

/** \brief New neighbor appeared callback in neighbor table.
* \param[out] MAC address of neighbor.
* \param[out] is neighbor provisioned or unprovisioned device.
* \param[out] uuid of neighbor. NULL if not available
* \param[out] network address of neighbor. MOBLE_ADDRESS_UNASSIGNED if not available
* \param[out] last updated rssi value.
*/
void BluenrgMesh_NeighborAppearedCallback(const MOBLEUINT8* bdAddr,
                                          MOBLEBOOL provisioned,
                                          const MOBLEUINT8* uuid,
                                          MOBLE_ADDRESS networkAddress,
                                          MOBLEINT8 rssi);

/** \brief Existing neighbor refreshed callback in neighbor table.
* \param[out] MAC address of neighbor.
* \param[out] is neighbor provisioned or unprovisioned device.
* \param[out] uuid of neighbor. NULL if not available
* \param[out] network address of neighbor. MOBLE_ADDRESS_UNASSIGNED if not available
* \param[out] last updated rssi value.
*/
void BluenrgMesh_NeighborRefreshedCallback(const MOBLEUINT8* bdAddr,
                                           MOBLEBOOL provisioned,
                                           const MOBLEUINT8* uuid,
                                           MOBLE_ADDRESS networkAddress,
                                           MOBLEINT8 rssi);

/** \brief Get neighbor table status.
* \param[in] pointer to application buff, it will be updated with neighbor table parameters.
* \param[in] reference to a variable which will be updated according to number of entries in neighbor table.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_GetNeighborState(neighbor_params_t* pNeighborTable, MOBLEUINT8* pNoOfNeighborPresent);

/** \brief Set system faults. Will be used by Health Model. Supporting All Bluetooth assigned FaultValues. 
* \param[in] pFaultArray FaultValue Array pointer. (FaultValue Range: 0x01�0x32)   
* \param[in] faultArraySize Size of the fault array. Max supported array size is 5. 
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_SetFault(MOBLEUINT8 *pFaultArray, MOBLEUINT8 faultArraySize);

/** \brief Clears already set system faults. Will be used by Health Model.  
* \param[in] pFaultArray Fault Array pointer  
* \param[in] faultArraySize Size of the fault array. Max supported array size is 5.
* \return MOBLE_RESULT_SUCCESS on success.
*/
MOBLE_RESULT BluenrgMesh_ClearFault(MOBLEUINT8 *pFaultArray, MOBLEUINT8 faultArraySize);

/** \brief Bluetooth LE Mesh Library shutdown
*
* This function should be called to shutdown Bluetooth LE Mesh Library
* To resume the operation, \a BluenrgMesh_Resume should be called.
* \return MOBLE_RESULT_FAIL if already shut down, MOBLE_RESULT_SUCCESS otherwise.
*/
MOBLE_RESULT BluenrgMesh_Shutdown(void);

/** \brief Restore Bluetooth LE Mesh Library after shutdown
*
* This function should be called to restore previously shutdown Bluetooth LE Mesh Library
* in order to resume library operation.
* \return MOBLE_RESULT_FAIL if already up and running, MOBLE_RESULT_SUCCESS otherwise.
*/
MOBLE_RESULT BluenrgMesh_Resume(void);
#endif /* __BLUENRG_MESH_ */

/******************* (C) COPYRIGHT 2017 STMicroelectronics *****END OF FILE****/

