/**
  ******************************************************************************
  * @file    usbd_imu.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @verbatim
  *
  *          ===================================================================
  *                                IMU Class  Description
  *          ===================================================================
  *
  *
  *
  *
  *
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_imu.h"
#include "usbd_ctlreq.h"

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_IMU
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_IMU_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_IMU_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_IMU_Private_Macros
  * @{
  */

/**
  * @}
  */




/** @defgroup USBD_IMU_Private_FunctionPrototypes
  * @{
  */


static uint8_t USBD_IMU_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_IMU_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_IMU_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_IMU_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_IMU_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_IMU_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_IMU_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_IMU_GetDeviceQualifierDescriptor(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */
/**
  * @}
  */

/** @defgroup USBD_IMU_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_IMU =
{
  USBD_IMU_Init,
  USBD_IMU_DeInit,
  USBD_IMU_Setup,
  NULL,
  NULL,
  USBD_IMU_DataIn,
  NULL,
  NULL,
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_IMU_GetHSCfgDesc,
  USBD_IMU_GetFSCfgDesc,
  USBD_IMU_GetOtherSpeedCfgDesc,
  USBD_IMU_GetDeviceQualifierDescriptor,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_IMU_CfgDesc[USB_IMU_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_IMU_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  0xC0,         /*bmAttributes: bus powered and Supports Remote Wakeup */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/
  /* 09 */

  /**********  Descriptor of IMU interface 0 Alternate setting 0 **************/

  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE  */

extern uint8_t IMU_InstID;
uint8_t* pTxIMUBuff = 0;
static uint8_t IMUInEpAdd = IMU_IN_EP;
static __IO uint8_t imu_ep_enabled = 0;
__IO uint8_t imu_ep_data = 0;
static uint16_t tx_imu_total_len = 0;
static uint16_t tx_imu_ptr = 0;

#ifndef USB_RAM_D2
#define USB_RAM_D2 __attribute__((section(".ram_d2")))
#endif

USB_RAM_D2 __ALIGN_BEGIN static uint8_t imu_tx_buffer[USB_IMU_MAX_SIZE] __ALIGN_END;

/**
  * @}
  */

/** @defgroup USBD_IMU_Private_Functions
  * @{
  */

/**
  * @brief  USBD_IMU_Init
  *         Initialize the IMU interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_IMU_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  IMUInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */
  printf("IMU_Init DATA IN EP: 0x%02X ClassID: 0x%02X\r\n", IMUInEpAdd, (uint8_t)pdev->classId);
  pTxIMUBuff = imu_tx_buffer;

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, IMUInEpAdd, USBD_EP_TYPE_BULK, IMU_HS_MAX_PACKET_SIZE);
  }
  else
  {
	/* Open EP IN */
	(void)USBD_LL_OpenEP(pdev, IMUInEpAdd, USBD_EP_TYPE_BULK, IMU_FS_MAX_PACKET_SIZE);
  }
  imu_ep_enabled = 1;

  pdev->ep_in[IMUInEpAdd & 0xFU].bInterval = 0;
  pdev->ep_in[IMUInEpAdd & 0xFU].is_used = 1U;

  /* #68: init-time ZLP removed — no protocol purpose, and a NULL-address
   * transmit wedges the IN EP in OTG DMA mode (see usbd_comms.c). */

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_IMU_DeInit
  *         DeInitialize the IMU layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_IMU_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this CDC class instance */
  IMUInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, IMUInEpAdd);
  pdev->ep_in[IMUInEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[IMUInEpAdd & 0xFU].total_length = 0U;
  imu_ep_enabled = 0;

  if(pTxIMUBuff){
    pTxIMUBuff = 0;
  }
#ifdef USE_USBD_COMPOSITE
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
	// ((USBD_HISTO_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
	(void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
	pdev->pClassDataCmsit[pdev->classId] = NULL;
	pdev->pClassData = NULL;
  }

#else
  /* Free memory */
  if (pdev->pClassData != NULL) {
	USBD_free(pdev->pClassData);
	pdev->pClassData = NULL;
  }
#endif
  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_IMU_Setup
  *         Handle the IMU specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_IMU_Setup(USBD_HandleTypeDef *pdev,
                                   USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  return (uint8_t)ret;
}


/**
  * @brief  USBD_IMU_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_IMU_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint8_t ret = USBD_OK;

#ifdef USE_USBD_COMPOSITE
	  /* Get the Endpoints addresses allocated for this CDC class instance */
	  IMUInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, IMU_InstID);
#endif /* USE_USBD_COMPOSITE */

  USBD_LL_FlushEP(pdev, IMUInEpAdd);

  if(imu_ep_data==1)
  {

      tx_imu_ptr += (pdev->dev_speed == USBD_SPEED_HIGH)?IMU_HS_MAX_PACKET_SIZE:IMU_FS_MAX_PACKET_SIZE;

      if (tx_imu_ptr < tx_imu_total_len)
      {
          uint16_t remaining = tx_imu_total_len - tx_imu_ptr;
          uint16_t pkt_len = MIN((pdev->dev_speed == USBD_SPEED_HIGH)?IMU_HS_MAX_PACKET_SIZE:IMU_FS_MAX_PACKET_SIZE, remaining);

          ret =  USBD_LL_Transmit(pdev, IMUInEpAdd, &pTxIMUBuff[tx_imu_ptr], pkt_len);
      }
      else
      {
    	  imu_ep_data = 0;
          USBD_IMU_TxCpltCallback(pTxIMUBuff, tx_imu_total_len, IMUInEpAdd);
		  /* Send ZLP */
		  // ret = USBD_LL_Transmit (pdev, IMUInEpAdd, NULL, 0U);
      }
  }else{
	pdev->ep_in[IMUInEpAdd & 0xFU].total_length = 0U;
	/* Send ZLP */
	ret = USBD_LL_Transmit (pdev, IMUInEpAdd, NULL, 0U);
  }

  return ret;
}


uint8_t USBD_IMU_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t len, uint8_t ep_idx)
{
  return (uint8_t)USBD_OK;
}

uint8_t  USBD_IMU_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t  *pbuff, uint16_t length)
{
	uint8_t ret = USBD_OK;
	if(imu_ep_enabled == 1 && imu_ep_data==0)
	{
#ifdef USE_USBD_COMPOSITE
	  /* Get the Endpoints addresses allocated for this CDC class instance */
	  IMUInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, IMU_InstID);
#endif /* USE_USBD_COMPOSITE */
	  USBD_LL_FlushEP(pdev, IMUInEpAdd);
	  memset((uint32_t*)pTxIMUBuff,0,USB_IMU_MAX_SIZE/4);
	  memcpy(pTxIMUBuff,pbuff,length);

      tx_imu_total_len = length;
      tx_imu_ptr = 0;

      uint16_t pkt_len = MIN((pdev->dev_speed == USBD_SPEED_HIGH)?IMU_HS_MAX_PACKET_SIZE:IMU_FS_MAX_PACKET_SIZE, tx_imu_total_len);

	  pdev->ep_in[IMUInEpAdd & 0xFU].total_length = tx_imu_total_len;
	  imu_ep_data = 1;

	  ret = USBD_LL_Transmit(pdev, IMUInEpAdd, pTxIMUBuff, (uint16_t)pkt_len);
	}
	else
	{
		ret = USBD_BUSY;
	}
  return ret;
}

uint8_t USBD_IMU_RegisterInterface(USBD_HandleTypeDef *pdev, uint8_t *buffer)
{
  UNUSED(pdev);
  UNUSED(buffer);
  return (uint8_t)USBD_OK;
}

__weak void USBD_IMU_TxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum)
{
	UNUSED(Buf);
	UNUSED(Len);
	UNUSED(epnum);
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_IMU_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_IMU_GetFSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = IMU_FS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = IMU_DATA_FS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = IMU_DATA_FS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_IMU_CfgDesc);
  return USBD_IMU_CfgDesc;
}

/**
  * @brief  USBD_IMU_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_IMU_GetHSCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = IMU_HS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = IMU_DATA_HS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = IMU_DATA_HS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_IMU_CfgDesc);
  return USBD_IMU_CfgDesc;
}

/**
  * @brief  USBD_IMU_GetOtherSpeedCfgDesc
  *         Return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_IMU_GetOtherSpeedCfgDesc(uint16_t *length)
{
  USBD_EpDescTypeDef *pEpCmdDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_CMD_EP);
  USBD_EpDescTypeDef *pEpOutDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_OUT_EP);
  USBD_EpDescTypeDef *pEpInDesc = USBD_GetEpDesc(USBD_IMU_CfgDesc, IMU_IN_EP);

  if (pEpCmdDesc != NULL)
  {
    pEpCmdDesc->bInterval = IMU_FS_BINTERVAL;
  }

  if (pEpOutDesc != NULL)
  {
    pEpOutDesc->wMaxPacketSize = IMU_DATA_FS_MAX_PACKET_SIZE;
  }

  if (pEpInDesc != NULL)
  {
    pEpInDesc->wMaxPacketSize = IMU_DATA_FS_MAX_PACKET_SIZE;
  }

  *length = (uint16_t)sizeof(USBD_IMU_CfgDesc);
  return USBD_IMU_CfgDesc;
}

/**
  * @brief  USBD_IMU_GetDeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
uint8_t *USBD_IMU_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_IMU_DeviceQualifierDesc);

  return USBD_IMU_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */
