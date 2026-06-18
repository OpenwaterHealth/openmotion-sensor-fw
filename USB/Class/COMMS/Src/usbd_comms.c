/**
  ******************************************************************************
  * @file    usbd_comms.c
  * @brief   Commsgram data streaming implementation
  ******************************************************************************
  */
#include "usbd_comms.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"
#include "utils.h"

/* Private typedef */
typedef struct {
  uint8_t *buffer;
  uint16_t length;
} comms_queue_entry_t;

#define COMMS_QUEUE_SIZE 4

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#define COMMS_TX_TIMEOUT_MS 250u

/* Private variables */
static comms_queue_entry_t comms_queue[COMMS_QUEUE_SIZE];
static uint8_t comms_queue_head = 0;
static uint8_t comms_queue_tail = 0;
static uint8_t comms_queue_count = 0;
static USBD_HandleTypeDef *comms_pdev = NULL;

static uint8_t comms_queue_enqueue(uint8_t *data, uint16_t length);
static uint8_t comms_queue_dequeue(uint8_t **data, uint16_t *length);
static uint8_t comms_queue_is_empty(void);
static uint8_t comms_queue_is_full(void);
static uint8_t comms_process_queue(void);

static uint8_t USBD_Comms_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Comms_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Comms_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_Comms_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_Comms_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_Comms_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_Comms_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_Comms_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_Comms_GetDeviceQualifierDescriptor(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */

USBD_ClassTypeDef USBD_COMMS = {
  USBD_Comms_Init,
  USBD_Comms_DeInit,
  USBD_Comms_Setup,
  NULL,                 /* EP0_TxSent */
  NULL,                 /* EP0_RxReady */
  USBD_Comms_DataIn,    /* DataIn */
  USBD_Comms_DataOut,   /* DataOut */
  NULL,                 /* SOF */
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_Comms_GetHSCfgDesc,
  USBD_Comms_GetFSCfgDesc,
  USBD_Comms_GetOtherSpeedCfgDesc,
  USBD_Comms_GetDeviceQualifierDescriptor,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_Comms_GetDeviceQualifierDescriptor[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
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

extern uint8_t COMMS_InstID;
static uint8_t* pTxCommsBuff = 0;
static uint16_t tx_comms_total_len = 0;
static uint16_t tx_comms_ptr = 0;
static __IO uint8_t comms_ep_enabled = 0;
__IO uint8_t comms_ep_data = 0;
static uint32_t comms_tx_start_ms = 0u;

static uint8_t COMMSInEpAdd = COMMS_IN_EP;
static uint8_t COMMSOutEpAdd = COMMS_OUT_EP;
static volatile uint32_t comms_enq_count = 0;
static volatile uint32_t comms_deq_count = 0;
static volatile uint32_t comms_datain_count = 0;
static volatile uint32_t comms_tx_fail_count = 0;

static uint16_t rxIndex = 0;

static void (*CommsRxCallback)(uint8_t *buf, uint16_t len) = NULL;
static uint8_t read_to_idle_enabled = 0;

#ifndef USB_RAM_D2
#define USB_RAM_D2 __attribute__((section(".ram_d2")))
#endif

USB_RAM_D2 __ALIGN_BEGIN static uint8_t comms_tx_buffer[USB_COMMS_MAX_SIZE] __ALIGN_END;
USB_RAM_D2 __ALIGN_BEGIN static uint8_t comms_queue_buffers[COMMS_QUEUE_SIZE][USB_COMMS_MAX_SIZE] __ALIGN_END;
USB_RAM_D2 __ALIGN_BEGIN static uint8_t rx_buffers[USB_RX_BUFFER_COUNT][USB_COMMS_MAX_SIZE] __ALIGN_END;
static uint8_t current_rx_buf_index = 0;

/* Private functions */
static uint8_t USBD_Comms_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  uint32_t packet_size = COMMS_FS_MAX_PACKET_SIZE;
  #ifdef USE_USBD_COMPOSITE
    /* Get the Endpoints addresses allocated for this class instance */
    COMMSInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
    COMMSOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  #endif /* USE_USBD_COMPOSITE */
    printf("COMMS_Init DATA IN EP: 0x%02X ClassID: 0x%02X\r\n", COMMSInEpAdd, (uint8_t)pdev->classId);
    printf("COMMS_Init DATA OUT EP: 0x%02X ClassID: 0x%02X\r\n", COMMSOutEpAdd, (uint8_t)pdev->classId);

    pTxCommsBuff = comms_tx_buffer;

    for (uint8_t i = 0; i < COMMS_QUEUE_SIZE; i++) {
      comms_queue[i].buffer = comms_queue_buffers[i];
      comms_queue[i].length = 0;
    }
    comms_queue_head = 0;
    comms_queue_tail = 0;
    comms_queue_count = 0;
    comms_pdev = pdev;

    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
		/* Open EPs */
		(void)USBD_LL_OpenEP(pdev, COMMSInEpAdd, USBD_EP_TYPE_BULK, COMMS_HS_MAX_PACKET_SIZE);
		(void)USBD_LL_OpenEP(pdev, COMMSOutEpAdd, USBD_EP_TYPE_BULK, COMMS_HS_MAX_PACKET_SIZE);
		packet_size = COMMS_HS_MAX_PACKET_SIZE;
    }
    else
    {
		/* Open EPs */
		(void)USBD_LL_OpenEP(pdev, COMMSInEpAdd, USBD_EP_TYPE_BULK, COMMS_FS_MAX_PACKET_SIZE);
		(void)USBD_LL_OpenEP(pdev, COMMSOutEpAdd, USBD_EP_TYPE_BULK, COMMS_FS_MAX_PACKET_SIZE);
		packet_size = COMMS_FS_MAX_PACKET_SIZE;
    }

    comms_ep_enabled = 1;
    pdev->ep_in[COMMSInEpAdd & 0xFU].bInterval = 0;
    pdev->ep_in[COMMSInEpAdd & 0xFU].is_used = 1U;

    memset((uint32_t*)rx_buffers, 0, (USB_COMMS_MAX_SIZE * USB_RX_BUFFER_COUNT)/4);
    current_rx_buf_index = 0;
    uint8_t *next_buffer = rx_buffers[current_rx_buf_index];
    rxIndex = 0;
    (void)USBD_LL_PrepareReceive(pdev, COMMSOutEpAdd, next_buffer, packet_size);

    /* Send ZLP */
    (void)USBD_LL_Transmit(pdev, COMMSInEpAdd, NULL, 0U);

    return (uint8_t)USBD_OK;
}

extern uint8_t COMMS_InstID;

static uint8_t USBD_Comms_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this CDC class instance */
  COMMSInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)COMMS_InstID);
#endif /* USE_USBD_COMPOSITE */

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, COMMSInEpAdd);
  pdev->ep_in[COMMSInEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[COMMSInEpAdd & 0xFU].total_length = 0U;
  comms_ep_enabled = 0;

  if(pTxCommsBuff != NULL){
    pTxCommsBuff = 0;
  }

  for (uint8_t i = 0; i < COMMS_QUEUE_SIZE; i++) {
    comms_queue[i].buffer = NULL;
  }
  comms_queue_head = 0;
  comms_queue_tail = 0;
  comms_queue_count = 0;
  comms_pdev = NULL;

#ifdef USE_USBD_COMPOSITE
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    // ((USBD_COMMS_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit();
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

/* Queue management functions */
static uint8_t comms_queue_is_empty(void)
{
  return (comms_queue_count == 0);
}

static uint8_t comms_queue_is_full(void)
{
  return (comms_queue_count >= COMMS_QUEUE_SIZE);
}

static uint8_t comms_queue_enqueue(uint8_t *data, uint16_t length)
{
  if (comms_queue_is_full() != 0) {
    printf("COMMS enqueue fail: queue full (count=%u size=%u enq=%lu deq=%lu datain=%lu txfail=%lu)\r\n",
           comms_queue_count, (uint8_t)COMMS_QUEUE_SIZE,
           (unsigned long)comms_enq_count,
           (unsigned long)comms_deq_count,
           (unsigned long)comms_datain_count,
           (unsigned long)comms_tx_fail_count);
    return USBD_FAIL;
  }

  if (length > USB_COMMS_MAX_SIZE) {
    printf("COMMS enqueue fail: length too large (%u > %u)\r\n",
           length, (uint16_t)USB_COMMS_MAX_SIZE);
    return USBD_FAIL;
  }

  memcpy(comms_queue[comms_queue_tail].buffer, data, length);
  comms_queue[comms_queue_tail].length = length;

  comms_queue_tail = (comms_queue_tail + 1) % COMMS_QUEUE_SIZE;
  comms_queue_count++;
  comms_enq_count++;

  return USBD_OK;
}

static uint8_t comms_queue_dequeue(uint8_t **data, uint16_t *length)
{
  if (comms_queue_is_empty() != 0) {
    return USBD_FAIL;
  }

  *data = comms_queue[comms_queue_head].buffer;
  *length = comms_queue[comms_queue_head].length;

  comms_queue_head = (comms_queue_head + 1) % COMMS_QUEUE_SIZE;
  comms_queue_count--;
  comms_deq_count++;

  return USBD_OK;
}

static uint8_t comms_process_queue(void)
{
  uint8_t *data;
  uint16_t length;

  if (comms_pdev == NULL) {
    return USBD_FAIL;
  }

  if (comms_queue_is_empty() != 0) {
    return USBD_OK;
  }

  if (comms_ep_data != 0) {
    return USBD_BUSY;
  }

  if (comms_queue_dequeue(&data, &length) != USBD_OK) {
    return USBD_FAIL;
  }

  return USBD_COMMS_SetTxBuffer(comms_pdev, data, length);
}

static uint8_t USBD_Comms_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  // Ignore everything, don't stall
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_Comms_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint8_t ret = USBD_OK;
  comms_datain_count++;

#ifdef USE_USBD_COMPOSITE
  COMMSInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, COMMS_InstID);
#endif /* USE_USBD_COMPOSITE */

  if (comms_ep_data == 1) {
    uint32_t pkt_size = (pdev->dev_speed == USBD_SPEED_HIGH) ? COMMS_HS_MAX_PACKET_SIZE : COMMS_FS_MAX_PACKET_SIZE;
    tx_comms_ptr += (uint16_t)pkt_size;

    if (tx_comms_ptr < tx_comms_total_len) {
      uint16_t remaining = tx_comms_total_len - tx_comms_ptr;
      uint16_t pkt_len = MIN((uint16_t)pkt_size, remaining);

      ret = USBD_LL_Transmit(pdev, COMMSInEpAdd, &pTxCommsBuff[tx_comms_ptr], pkt_len);
      if (ret != USBD_OK) {
        comms_tx_fail_count++;
        comms_ep_data = 0;
        comms_tx_start_ms = 0u;
      } else {
        comms_tx_start_ms = get_timestamp_ms();
      }
    } else {
      /* Transfer complete */
      comms_ep_data = 0;
      comms_tx_start_ms = 0u;
      USBD_COMMS_TxCpltCallback(pTxCommsBuff, tx_comms_total_len, COMMSInEpAdd);

      /* Process queue to send next packet if available */
      comms_process_queue();
    }
  } else {
    pdev->ep_in[COMMSInEpAdd & 0xFU].total_length = 0U;
    /* Send ZLP */
    ret = USBD_LL_Transmit(pdev, COMMSInEpAdd, NULL, 0U);
  }
  return ret;
}

static uint8_t USBD_Comms_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_StatusTypeDef status;
#ifdef USE_USBD_COMPOSITE
  COMMSOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_BULK, COMMS_InstID);
#endif

  uint8_t *buf = rx_buffers[current_rx_buf_index];
  uint8_t *next_buffer = buf; //set to current buffer unless we switch it
  uint32_t rx_len = USBD_LL_GetRxDataSize(pdev, COMMSOutEpAdd);
  uint32_t packet_size = (pdev->dev_speed == USBD_SPEED_HIGH) ?
                         COMMS_HS_MAX_PACKET_SIZE : COMMS_FS_MAX_PACKET_SIZE;

  /* Safe buffer handling */
  if(rxIndex + rx_len <= USB_COMMS_MAX_SIZE)
  {
    rxIndex += rx_len;

    /* Determine packet completion based on length field */
    if (rxIndex >= 9)
    {
      uint16_t data_len = (uint16_t)((buf[7] << 8) | buf[8]);
      uint32_t total_packet_size = 9U + data_len + 3U;  // header + data + CRC(2) + END(1)

      if (total_packet_size > USB_COMMS_MAX_SIZE)
      {
        /* Invalid length, reset */
        rxIndex = 0;
        current_rx_buf_index ^= 1;
        next_buffer = rx_buffers[current_rx_buf_index];
        memset((uint32_t*)next_buffer, 0, USB_COMMS_MAX_SIZE/4);
      }
      else if (rxIndex >= total_packet_size)
      {
        /* Complete packet received */
        USBD_COMMS_RxCpltCallback(buf, (uint16_t)total_packet_size, COMMSOutEpAdd);
        rxIndex = 0;
        current_rx_buf_index ^= 1; // switch buffer
        next_buffer = rx_buffers[current_rx_buf_index];
        memset((uint32_t*)next_buffer, 0, USB_COMMS_MAX_SIZE/4);
      }
    }
  }
  else
  {
    /* Buffer overflow */
    rxIndex = 0;
    current_rx_buf_index ^= 1; // switch buffer
    next_buffer = rx_buffers[current_rx_buf_index];
    memset((uint32_t*)next_buffer, 0, USB_COMMS_MAX_SIZE/4);
  }

  status = USBD_LL_PrepareReceive(pdev, COMMSOutEpAdd, &next_buffer[rxIndex], packet_size);
  return status;
}

uint8_t USBD_COMMS_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t len, uint8_t ep_idx)
{
  UNUSED(ep_idx);

  if (pdev == NULL) {
    printf("COMMS SendData fail: pdev NULL\r\n");
    return USBD_FAIL;
  }

  if (data == NULL) {
    printf("COMMS SendData fail: data NULL\r\n");
    return USBD_FAIL;
  }

  if (len == 0) {
    printf("COMMS SendData fail: len=0\r\n");
    return USBD_FAIL;
  }

  if (len > USB_COMMS_MAX_SIZE) {
    printf("COMMS SendData fail: len too large (%u > %u)\r\n",
           len, (uint16_t)USB_COMMS_MAX_SIZE);
    return USBD_FAIL;
  }

  if (comms_pdev == NULL) {
    comms_pdev = pdev;
  }

  /* If queue is empty and not currently sending, send directly */
  if (comms_queue_is_empty() && comms_ep_data == 0) {
    uint8_t ret = USBD_COMMS_SetTxBuffer(pdev, data, len);
    if (ret != USBD_OK) {
      printf("COMMS SendData fail: SetTxBuffer ret=%u (ep_data=%u enabled=%u)\r\n",
             ret, comms_ep_data, comms_ep_enabled);
    }
    return ret;
  }

  if (comms_ep_data == 0 && !comms_queue_is_empty()) {
    (void)comms_process_queue();
  }

  /* Otherwise, add to queue */
  return comms_queue_enqueue(data, len);
}

uint8_t  USBD_COMMS_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t  *pbuff, uint16_t length)
{
  uint8_t ret = USBD_OK;

  if (pdev == NULL) {
    printf("USBD_COMMS_SetTxBuffer: pdev NULL\r\n");
    return USBD_FAIL;
  }

  if (comms_ep_enabled == 1 && comms_ep_data == 0) {
#ifdef USE_USBD_COMPOSITE
    COMMSInEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, COMMS_InstID);
#endif /* USE_USBD_COMPOSITE */

    USBD_LL_FlushEP(pdev, COMMSInEpAdd);
    memset((uint32_t*)pTxCommsBuff, 0, USB_COMMS_MAX_SIZE / 4);
    memcpy(pTxCommsBuff, pbuff, length);

    tx_comms_total_len = length;
    tx_comms_ptr = 0;

    uint16_t pkt_len = MIN((pdev->dev_speed == USBD_SPEED_HIGH) ? COMMS_HS_MAX_PACKET_SIZE : COMMS_FS_MAX_PACKET_SIZE, tx_comms_total_len);

    pdev->ep_in[COMMSInEpAdd & 0xFU].total_length = tx_comms_total_len;

    ret = USBD_LL_Transmit(pdev, COMMSInEpAdd, pTxCommsBuff, pkt_len);
    if (ret == USBD_OK) {
      comms_ep_data = 1;
      comms_tx_start_ms = get_timestamp_ms();
    } else {
      comms_tx_fail_count++;
      comms_ep_data = 0;
    }
  } else {
    ret = USBD_BUSY;
  }
  return ret;
}


uint8_t USBD_COMMS_Transmit(USBD_HandleTypeDef *pdev, uint8_t* Buf, uint16_t Len)
{
	return USBD_COMMS_SetTxBuffer(pdev, Buf, Len);
}

void USBD_COMMS_FlushRxBuffer()
{

}

uint8_t USBD_COMMS_RegisterRxCallback(void (*cb)(uint8_t *buf, uint16_t len)) {
    CommsRxCallback = cb;
    return USBD_OK;
}

uint8_t USBD_COMMS_RegisterInterface(USBD_HandleTypeDef *pdev, uint8_t *buffer)
{
  UNUSED(pdev);
  UNUSED(buffer);
  return (uint8_t)USBD_OK;
}

extern USBD_HandleTypeDef hUsbDeviceHS;

void USBD_COMMS_RecoverFromError(void)
{
  printf("USBD_COMMS_RecoverFromError\r\n");
  __disable_irq();
  USBD_LL_FlushEP(&hUsbDeviceHS, COMMS_IN_EP);
  USBD_LL_FlushEP(&hUsbDeviceHS, COMMS_OUT_EP);

  tx_comms_ptr = 0;
  tx_comms_total_len = 0;
  comms_ep_data = 0;
  rxIndex = 0;
  read_to_idle_enabled = 0;

  // Reinitialize endpoints
  USBD_LL_OpenEP(&hUsbDeviceHS, COMMS_IN_EP,
                USBD_EP_TYPE_BULK, COMMS_FS_MAX_PACKET_SIZE);
  uint8_t *next_buffer = rx_buffers[current_rx_buf_index];
  memset((uint32_t*)next_buffer, 0, USB_COMMS_MAX_SIZE/4);
  USBD_LL_PrepareReceive(&hUsbDeviceHS, COMMS_OUT_EP,
		  next_buffer, COMMS_FS_MAX_PACKET_SIZE);
  __enable_irq();
}

__weak void USBD_COMMS_TxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum)
{
	UNUSED(Buf);
	UNUSED(Len);
	UNUSED(epnum);
}

__weak void USBD_COMMS_RxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum)
{
	UNUSED(Buf);
	UNUSED(Len);
	UNUSED(epnum);
}
