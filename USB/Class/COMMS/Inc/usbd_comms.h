/**
  ******************************************************************************
  * @file    usbd_comms.h
  * @brief   Header file for USB comms
  ******************************************************************************
  */
#ifndef __USB_COMMS_H
#define __USB_COMMS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_ioreq.h"
#include "common.h"


#ifndef COMMS_IN_EP
#define COMMS_IN_EP                                   0x81U  /* EP1 for data IN */
#endif /* HISTO_IN_EP */

#ifndef COMMS_OUT_EP
#define COMMS_OUT_EP                        0x01U  /* EP1 for data OUT */
#endif

#define COMMS_FS_MAX_PACKET_SIZE         64U    /* Full-speed USB */
#define COMMS_HS_MAX_PACKET_SIZE         512U   /* High-speed USB */
#ifndef USB_COMMS_MAX_SIZE
#define USB_COMMS_MAX_SIZE                COMMAND_MAX_SIZE
#endif
#define USB_RX_BUFFER_COUNT     		 2U
extern USBD_ClassTypeDef USBD_COMMS;
#define USBD_COMMS_CLASS &USBD_COMMS

uint8_t  USBD_COMMS_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t  *pbuff, uint16_t length);
uint8_t USBD_COMMS_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t len, uint8_t ep_idx);
uint8_t USBD_COMMS_Transmit(USBD_HandleTypeDef *pdev, uint8_t* Buf, uint16_t Len);
void USBD_COMMS_FlushRxBuffer(void);
uint8_t USBD_COMMS_RegisterInterface(USBD_HandleTypeDef *pdev, uint8_t *buffer);
uint8_t USBD_COMMS_RegisterRxCallback(void (*cb)(uint8_t *buf, uint16_t len));
void USBD_COMMS_RxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum);
void USBD_COMMS_TxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum);
void USBD_COMMS_RecoverFromError(void);
#ifdef __cplusplus
}
#endif

#endif /* __USB_COMMS_H */
