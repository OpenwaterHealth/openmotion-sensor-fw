/**
  ******************************************************************************
  * @file    usbd_histo.c
  * @brief   Histogram data streaming implementation
  ******************************************************************************
  */
#include "usbd_histo.h"
#include "usbd_ctlreq.h"
#include "usbd_desc.h"
#include "common.h"
#include "logging.h"
#include "utils.h"

#define HISTO_THROTTLE_INTERVAL_MS 5000u

/* Private typedef */
typedef struct {
  uint8_t *buffer;
  uint16_t length;
} histo_queue_entry_t;

#define HISTO_QUEUE_SIZE 4  /* Reduced from 8 to save memory (8 * 36KB = 288KB was too much) */

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

/* Private variables.
 * head/tail/count are shared between the frame ISR (enqueue, via
 * USBD_HISTO_SendData) and the USB OTG_HS ISR (dequeue, via USBD_Histo_DataIn).
 * On the external-FSIN path the USB ISR (NVIC prio 0) can PREEMPT the frame ISR
 * (prio 2) mid read-modify-write, so these must be volatile AND every mutation
 * runs in a __disable_irq critical section (see enqueue/dequeue). Without that,
 * a lost count++/count-- drifts the count, stranding a never-dequeued slot whose
 * stale bytes re-ship into a later scan. */
static histo_queue_entry_t histo_queue[HISTO_QUEUE_SIZE];
static volatile uint8_t histo_queue_head = 0;
static volatile uint8_t histo_queue_tail = 0;
static volatile uint8_t histo_queue_count = 0;
static USBD_HandleTypeDef *histo_pdev = NULL;

/* Private function prototypes */
static uint8_t histo_queue_enqueue(uint8_t *data, uint16_t length);
static uint8_t histo_queue_dequeue(uint8_t **data, uint16_t *length);
static uint8_t histo_queue_is_empty(void);
static uint8_t histo_queue_is_full(void);
static uint8_t histo_process_queue(void);
static uint8_t USBD_Histo_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Histo_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_Histo_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static uint8_t USBD_Histo_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_Histo_GetFSCfgDesc(uint16_t *length);
static uint8_t *USBD_Histo_GetHSCfgDesc(uint16_t *length);
static uint8_t *USBD_Histo_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_Histo_GetDeviceQualifierDescriptor(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */

USBD_ClassTypeDef USBD_HISTO = {
  USBD_Histo_Init,
  USBD_Histo_DeInit,
  USBD_Histo_Setup,
  NULL,                 /* EP0_TxSent */
  NULL,                 /* EP0_RxReady */
  USBD_Histo_DataIn,    /* DataIn */
  NULL,                 /* DataOut */
  NULL,                 /* SOF */
  NULL,
  NULL,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_Histo_GetHSCfgDesc,
  USBD_Histo_GetFSCfgDesc,
  USBD_Histo_GetOtherSpeedCfgDesc,
  USBD_Histo_GetDeviceQualifierDescriptor,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_Histo_GetDeviceQualifierDescriptor[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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

extern uint8_t HISTO_InstID;
static uint8_t* pTxHistoBuff = 0;
static volatile uint16_t tx_histo_total_len = 0;
static volatile uint16_t tx_histo_ptr = 0;
static __IO uint8_t histo_ep_enabled = 0;
__IO uint8_t histo_ep_data = 0;
static uint8_t HISTOInEpAdd = HISTO_IN_EP;
static volatile uint32_t histo_enq_count = 0;
static volatile uint32_t histo_deq_count = 0;
static volatile uint32_t histo_datain_count = 0;
static volatile uint32_t histo_tx_fail_count = 0;

#ifndef USB_RAM_D2
#define USB_RAM_D2 __attribute__((section(".ram_d2")))
#endif

USB_RAM_D2 __ALIGN_BEGIN static uint8_t histo_tx_buffer[USB_HISTO_MAX_SIZE] __ALIGN_END;
USB_RAM_D2 __ALIGN_BEGIN static uint8_t histo_queue_buffers[HISTO_QUEUE_SIZE][USB_HISTO_MAX_SIZE] __ALIGN_END;

/* Private functions */
static uint8_t USBD_Histo_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  uint8_t ret = USBD_OK;
  UNUSED(cfgidx);

  #ifdef USE_USBD_COMPOSITE
    /* Get the Endpoints addresses allocated for this class instance */
    HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)pdev->classId);
  #endif /* USE_USBD_COMPOSITE */
    printf("HISTO_Init DATA IN EP: 0x%02X ClassID: 0x%02X\r\n", HISTOInEpAdd, (uint8_t)pdev->classId);
    pTxHistoBuff = histo_tx_buffer;

    /* Allocate buffers for queue entries */
    for (uint8_t i = 0; i < HISTO_QUEUE_SIZE; i++) {
      histo_queue[i].buffer = histo_queue_buffers[i];
      histo_queue[i].length = 0;
    }

    /* Initialize queue */
    histo_queue_head = 0;
    histo_queue_tail = 0;
    histo_queue_count = 0;
    histo_pdev = pdev;

    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {
      /* Open EP IN */
      (void)USBD_LL_OpenEP(pdev, HISTOInEpAdd, USBD_EP_TYPE_BULK, HISTO_HS_MAX_PACKET_SIZE);

    }
    else
    {
    /* Open EP IN */
    (void)USBD_LL_OpenEP(pdev, HISTOInEpAdd, USBD_EP_TYPE_BULK, HISTO_FS_MAX_PACKET_SIZE);
    }
    histo_ep_enabled = 1;
    pdev->ep_in[HISTOInEpAdd & 0xFU].bInterval = 0;
    pdev->ep_in[HISTOInEpAdd & 0xFU].is_used = 1U;

	/* #68: init-time ZLP removed — no protocol purpose, and a NULL-address
	 * transmit wedges the IN EP in OTG DMA mode (see usbd_comms.c). */

    return ret;
}

extern uint8_t HISTO_InstID;

static uint8_t USBD_Histo_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this CDC class instance */
  HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, (uint8_t)HISTO_InstID);
#endif /* USE_USBD_COMPOSITE */

  /* Close EP IN */
  (void)USBD_LL_CloseEP(pdev, HISTOInEpAdd);
  pdev->ep_in[HISTOInEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[HISTOInEpAdd & 0xFU].total_length = 0U;
  histo_ep_enabled = 0;

  if(pTxHistoBuff != NULL){
    pTxHistoBuff = 0;
  }

  /* Free queue entry buffers */
  for (uint8_t i = 0; i < HISTO_QUEUE_SIZE; i++) {
    histo_queue[i].buffer = NULL;
  }

  histo_queue_head = 0;
  histo_queue_tail = 0;
  histo_queue_count = 0;
  histo_pdev = NULL;
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

/* Queue management functions */
static uint8_t histo_queue_is_empty(void)
{
  return (histo_queue_count == 0);
}

static uint8_t histo_queue_is_full(void)
{
  return (histo_queue_count >= HISTO_QUEUE_SIZE);
}

static uint8_t histo_queue_enqueue(uint8_t *data, uint16_t length)
{
  if (histo_queue_is_full() != 0) {
    printf("HISTO enqueue fail: queue full (count=%u size=%u enq=%lu deq=%lu datain=%lu txfail=%lu)\r\n",
           histo_queue_count, (uint8_t)HISTO_QUEUE_SIZE,
           (unsigned long)histo_enq_count,
           (unsigned long)histo_deq_count,
           (unsigned long)histo_datain_count,
           (unsigned long)histo_tx_fail_count);
    return USBD_FAIL;
  }

  if (length > USB_HISTO_MAX_SIZE) {
    printf("HISTO enqueue fail: length too large (%u > %u)\r\n",
           length, (uint16_t)USB_HISTO_MAX_SIZE);
    return USBD_FAIL;
  }

  /* Copy into the current tail slot BEFORE publishing it. enqueue runs only in
   * the frame ISR (non-reentrant), so `tail` is private here and the slot stays
   * invisible to the dequeue path until count++ commits below. The 32 KB memcpy
   * is deliberately OUTSIDE the critical section — never hold off IRQs across it. */
  uint8_t slot = histo_queue_tail;
  memcpy(histo_queue[slot].buffer, data, length);
  histo_queue[slot].length = length;

  /* Publish atomically: `count` is shared with the dequeue path (USB ISR), which
   * can preempt this frame-ISR enqueue, so the RMW must not be interruptible. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  histo_queue_tail = (uint8_t)((slot + 1) % HISTO_QUEUE_SIZE);
  histo_queue_count++;
  histo_enq_count++;
  __set_PRIMASK(primask);

  return USBD_OK;
}

static uint8_t histo_queue_dequeue(uint8_t **data, uint16_t *length)
{
  /* Empty-check + head advance + count-- as one atomic step: dequeue is called
   * from BOTH the frame ISR and the USB ISR, so two concurrent dequeues must not
   * both claim the same head slot. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (histo_queue_count == 0) {
    __set_PRIMASK(primask);
    return USBD_FAIL;
  }
  uint8_t slot = histo_queue_head;
  histo_queue_head = (uint8_t)((slot + 1) % HISTO_QUEUE_SIZE);
  histo_queue_count--;
  histo_deq_count++;
  __set_PRIMASK(primask);

  *data = histo_queue[slot].buffer;
  *length = histo_queue[slot].length;

  return USBD_OK;
}

static uint8_t histo_process_queue(void)
{
  uint8_t *data;
  uint16_t length;

  if (histo_pdev == NULL) {
    return USBD_FAIL;
  }

  if (histo_queue_is_empty() != 0) {
    return USBD_OK;
  }

  /* If still sending, don't process queue yet */
  if (histo_ep_data != 0) {
    return USBD_BUSY;
  }

  /* Dequeue next packet */
  if (histo_queue_dequeue(&data, &length) != USBD_OK) {
    return USBD_FAIL;
  }

  /* Send the dequeued packet */
  return USBD_HISTO_SetTxBuffer(histo_pdev, data, length);
}

static uint8_t USBD_Histo_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  // Ignore everything, don't stall
  return (uint8_t)USBD_OK;
}

static uint8_t USBD_Histo_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
	uint8_t ret = USBD_OK;
  histo_datain_count++;

#ifdef USE_USBD_COMPOSITE
	  /* Get the Endpoints addresses allocated for this CDC class instance */
	HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, HISTO_InstID);
#endif /* USE_USBD_COMPOSITE */

  if(histo_ep_data==1){
      tx_histo_ptr += (pdev->dev_speed == USBD_SPEED_HIGH)?HISTO_HS_MAX_PACKET_SIZE:HISTO_FS_MAX_PACKET_SIZE;

      if (tx_histo_ptr < tx_histo_total_len)
      {
          uint16_t remaining = tx_histo_total_len - tx_histo_ptr;
          uint16_t pkt_len = MIN((pdev->dev_speed == USBD_SPEED_HIGH)?HISTO_HS_MAX_PACKET_SIZE:HISTO_FS_MAX_PACKET_SIZE, remaining);

          ret =  USBD_LL_Transmit(pdev, HISTOInEpAdd, &pTxHistoBuff[tx_histo_ptr], pkt_len);
          if (ret != USBD_OK) {
            histo_tx_fail_count++;
            histo_ep_data = 0;
            /* Mid-transfer failure: resume draining so a tx error doesn't wedge
             * the EP and strand the rest of the queue until the next scan. */
            histo_process_queue();
          }
      }
      else
      {
          // Transfer complete
          histo_ep_data = 0;
          USBD_HISTO_TxCpltCallback(pTxHistoBuff, tx_histo_total_len, HISTOInEpAdd);
		  
		  /* Process queue to send next packet if available */
		  histo_process_queue();
      }
  }else{
	pdev->ep_in[HISTOInEpAdd & 0xFU].total_length = 0U;
	/* Send ZLP */
	ret = USBD_LL_Transmit (pdev, HISTOInEpAdd, NULL, 0U);
  }

  return ret;
}

/* Last timestamp when a histogram packet was actually sent (for DEBUG_FLAG_HISTO_THROTTLE) */
static uint32_t histo_last_send_ms = 0;

uint8_t USBD_HISTO_SendData(USBD_HandleTypeDef *pdev, uint8_t *data, uint16_t len, uint8_t ep_idx)
{
  UNUSED(ep_idx);
  
  if (pdev == NULL) {
    printf("HISTO SendData fail: pdev NULL\r\n");
    return USBD_FAIL;
  }

  if (data == NULL) {
    printf("HISTO SendData fail: data NULL\r\n");
    return USBD_FAIL;
  }

  if (len == 0) {
    printf("HISTO SendData fail: len=0\r\n");
    return USBD_FAIL;
  }

  if (len > USB_HISTO_MAX_SIZE) {
    printf("HISTO SendData fail: len too large (%u > %u)\r\n",
           len, (uint16_t)USB_HISTO_MAX_SIZE);
    return USBD_FAIL;
  }

  /* Debug flag: only send histogram packet every 5 seconds; others pretend success */
  if ((logging_get_debug_flags() & DEBUG_FLAG_HISTO_THROTTLE) != 0u) {
    uint32_t now_ms = get_timestamp_ms();
    uint32_t elapsed = (histo_last_send_ms != 0u) ? (now_ms - histo_last_send_ms) : HISTO_THROTTLE_INTERVAL_MS;
    if (elapsed < HISTO_THROTTLE_INTERVAL_MS) {
      return USBD_OK;  /* Pretend sent, do not enqueue or transmit */
    }
    histo_last_send_ms = now_ms;
  }

  /* Ensure pdev is stored (in case called before Init, though this shouldn't happen) */
  if (histo_pdev == NULL) {
    histo_pdev = pdev;
  }

  /* If queue is empty and not currently sending, send directly */
  if (histo_queue_is_empty() && histo_ep_data == 0) {
    uint8_t ret = USBD_HISTO_SetTxBuffer(pdev, data, len);
    if (ret != USBD_OK) {
      printf("HISTO SendData fail: SetTxBuffer ret=%u (ep_data=%u enabled=%u)\r\n",
             ret, histo_ep_data, histo_ep_enabled);
    }
    return ret;
  }
  
  if (histo_ep_data == 0 && !histo_queue_is_empty()) {
    (void)histo_process_queue();
  }

  /* Otherwise, add to queue */
  return histo_queue_enqueue(data, len);
}

/* Drop any histogram packets left in the software TX queue (and the
 * hardware EP FIFO) from a previous scan.  The queue is otherwise only
 * reset on USB (de)enumeration, so packets still queued when a scan stops
 * drain into the NEXT scan carrying that scan's TIM5 timestamp and
 * frame_id — the host renders them as negative scan-relative timestamps
 * and out-of-order frame_ids (the leftover-frame / "stale frame" bug).
 * Call this at scan start (OW_CAMERA_STREAM enable) BEFORE arming the
 * cameras, so frame 1 of the new scan is the first packet the host sees.
 * Touches state shared with the frame ISR (USBD_HISTO_SendData) and the
 * USB ISR (USBD_Histo_DataIn), so it runs in a critical section. */
void USBD_HISTO_FlushQueue(const char *who)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  /* Snapshot what was still in the queue before we clear it. */
  uint8_t pending  = histo_queue_count;
  uint8_t inflight = histo_ep_data;
  long    gap      = (long)histo_enq_count - (long)histo_deq_count;

  /* Empty the software queue + abandon any in-flight transfer + drop the EP
   * FIFO, so nothing from this scan can carry into the next. */
  histo_queue_head = 0;
  histo_queue_tail = 0;
  histo_queue_count = 0;
  histo_ep_data = 0;
  tx_histo_ptr = 0;
  tx_histo_total_len = 0;
  if (histo_pdev != NULL && histo_ep_enabled != 0) {
    USBD_LL_FlushEP(histo_pdev, HISTOInEpAdd);
  }

  __set_PRIMASK(primask);

  /* Diagnostic (short, single-chunk — the USB printf channel splits long lines):
   *   q  = frames still queued at the boundary (the leftover source)
   *   ep = a transfer was still in flight
   *   e-d= enq-deq; this should EQUAL q. If e-d != q the count was corrupted by
   *        a cross-ISR race; if e-d == q > 0 the scan honestly left frames unsent. */
  printf("HISTO flush(%s): q=%u ep=%u e-d=%ld\r\n", who, pending, inflight, gap);
}

uint8_t  USBD_HISTO_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t  *pbuff, uint16_t length)
{
	uint8_t ret = USBD_OK;

	if(histo_ep_enabled == 1 && histo_ep_data==0)
	{
#ifdef USE_USBD_COMPOSITE
		/* Get the Endpoints addresses allocated for this CDC class instance */
		HISTOInEpAdd  = USBD_CoreGetEPAdd(pdev, USBD_EP_IN, USBD_EP_TYPE_BULK, HISTO_InstID);
#endif /* USE_USBD_COMPOSITE */

		USBD_LL_FlushEP(pdev, HISTOInEpAdd);
		memset((uint32_t*)pTxHistoBuff,0,USB_HISTO_MAX_SIZE/4);
		memcpy(pTxHistoBuff,pbuff,length);

        tx_histo_total_len = length;
        tx_histo_ptr = 0;

        uint16_t pkt_len = MIN((pdev->dev_speed == USBD_SPEED_HIGH)?HISTO_HS_MAX_PACKET_SIZE:HISTO_FS_MAX_PACKET_SIZE, tx_histo_total_len);

		pdev->ep_in[HISTOInEpAdd & 0xFU].total_length = tx_histo_total_len;

		ret = USBD_LL_Transmit(pdev, HISTOInEpAdd, pTxHistoBuff, pkt_len);
		if (ret == USBD_OK) {
			histo_ep_data = 1;
		} else {
			histo_tx_fail_count++;
			histo_ep_data = 0;
		}
	}
	else
	{
		ret = USBD_BUSY;
	}
  return ret;
}

static uint8_t USBD_HISTO_RegisterInterface(USBD_HandleTypeDef *pdev, uint8_t *buffer)
{
  UNUSED(pdev);
  UNUSED(buffer);
  return (uint8_t)USBD_OK;
}

__weak void USBD_HISTO_TxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum)
{
	UNUSED(Buf);
	UNUSED(Len);
	UNUSED(epnum);
}
