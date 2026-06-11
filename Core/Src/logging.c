#include "main.h"
#include "logging.h"
#include "lwrb.h"
#include "uart_comms.h"
#include "common.h"
#include "usbd_comms.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

static bool bInit_dma = false;
volatile bool bPrintfTransferComplete = false;
static volatile uint32_t debug_flags = 0x00000000;

static uint8_t usart_start_tx_dma_transfer(void);
static bool logging_uart_dma_write(const uint8_t *src, size_t count);

/* Ring buffer for TX data */
lwrb_t usart_tx_buff;
uint8_t usart_tx_buff_data[1512];
volatile size_t usart_tx_dma_current_len;
static bool usart_drop_marker_pending = false;

/* Ring buffer for accumulating printf messages to send over command and control endpoint */
#define LOG_MSG_BUFFER_SIZE 2048
static lwrb_t log_msg_rb;
static uint8_t log_msg_buffer[LOG_MSG_BUFFER_SIZE];
static bool log_msg_rb_initialized = false;
static bool log_msg_drop_marker_pending = false;

// External reference to tx_flag to check if transmission is in progress
extern volatile uint8_t tx_flag;

/* Forward declarations */
static void send_log_message_over_comms(void);
static void add_char_to_log_buffer(uint8_t ch);

#ifdef __GNUC__
	/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
		 set to 'Yes') calls __io_putchar() */
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


#ifdef __GNUC__
int _write(int fd, const void *buf, size_t count){
	UNUSED(fd);
	uint8_t * src = (uint8_t *)buf;
	
	// Send to UART (existing functionality)
	if(bInit_dma)
	{
		(void)logging_uart_dma_write(src, count);
	}
	else
	{
		HAL_StatusTypeDef com_tx_status = HAL_UART_Transmit(&DEBUG_UART, src, count, 10);
		if(com_tx_status != HAL_OK)
		{
			Error_Handler();
		}
	}

	// Also send to command and control endpoint (USB) if enabled
	if ((debug_flags & DEBUG_FLAG_USB_PRINTF) != 0u) {
		for (size_t i = 0; i < count; i++) {
			add_char_to_log_buffer(src[i]);
		}
	}

	return count;
}
#else
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
	//HAL_UART_Transmit(&DEBUG_UART, (uint8_t *)&ch, 1,10);
	UART_putChar( (uint8_t) ch);
  return ch;
}
#endif

void init_dma_logging()
{
    /* Initialize ringbuff */
    lwrb_init(&usart_tx_buff, usart_tx_buff_data, sizeof(usart_tx_buff_data));

    bInit_dma = true;
	bPrintfTransferComplete = true;
}

bool is_using_dma(){
	return bInit_dma;
}

void logging_set_debug_flags(uint32_t flags) {
	uint32_t prev_flags = debug_flags;
	debug_flags = flags;
	if (prev_flags != debug_flags) {
		printf("Debug flags changed: 0x%08lX -> 0x%08lX%s%s%s%s%s\r\n",
		       (unsigned long)prev_flags,
		       (unsigned long)debug_flags,
		       (debug_flags & DEBUG_FLAG_USB_PRINTF) ? " (USB printf ON)" : " (USB printf OFF)",
		       (debug_flags & DEBUG_FLAG_HISTO_SPARSE) ? " (HISTO sparse ON)" : "",
		       (debug_flags & DEBUG_FLAG_COMM_VERBOSE) ? " (comm verbose ON)" : "",
		       (debug_flags & DEBUG_FLAG_CMD_VERBOSE) ? " (verbose cmd ON)" : "",
		       (debug_flags & DEBUG_FLAG_HISTO_CMP) ? " (histo compress ON)" : "");
	}
	if ((debug_flags & DEBUG_FLAG_USB_PRINTF) == 0u) {
		// Drop any buffered log data so it doesn't flush later.
		if (log_msg_rb_initialized) {
			lwrb_reset(&log_msg_rb);
		}
	}
}

uint32_t logging_get_debug_flags(void) {
	return debug_flags;
}

static bool logging_uart_dma_write(const uint8_t *src, size_t count) {
	if (count == 0) {
		return true;
	}

	size_t remaining = count;
	uint32_t start_time = get_timestamp_ms();
	const uint32_t timeout_ms = 10u;

	while (remaining > 0) {
		size_t free_bytes = lwrb_get_free(&usart_tx_buff);
		if (free_bytes == 0) {
			usart_start_tx_dma_transfer();
			if ((get_timestamp_ms() - start_time) >= timeout_ms) {
				usart_drop_marker_pending = true;
				break;
			}
			continue;
		}

		if (usart_drop_marker_pending && free_bytes > 1) {
			const uint8_t marker = '~';
			lwrb_write(&usart_tx_buff, &marker, 1);
			usart_drop_marker_pending = false;
			free_bytes--;
		}

		if (free_bytes == 0) {
			continue;
		}

		size_t to_write = remaining < free_bytes ? remaining : free_bytes;
		lwrb_write(&usart_tx_buff, src, to_write);
		src += to_write;
		remaining -= to_write;
		usart_start_tx_dma_transfer();
	}

	return remaining == 0;
}

static uint8_t usart_start_tx_dma_transfer(void) {
    if (usart_tx_dma_current_len == 0 && (usart_tx_dma_current_len = lwrb_get_linear_block_read_length(&usart_tx_buff)) > 0) {

        /* Limit maximal size to transmit at a time */
        if (usart_tx_dma_current_len > 32) {
            usart_tx_dma_current_len = 32;
        }
    	bPrintfTransferComplete = false;
		if(HAL_UART_Transmit_DMA(&DEBUG_UART, (uint8_t*)lwrb_get_linear_block_read_address(&usart_tx_buff), usart_tx_dma_current_len)!= HAL_OK)
		{
			Error_Handler();
		}
    }
    return 1;
}

void logging_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{

}


void logging_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}


void logging_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	bPrintfTransferComplete = true;
    lwrb_skip(&usart_tx_buff, usart_tx_dma_current_len);/* Data sent, ignore these */
    usart_tx_dma_current_len = 0;
    usart_start_tx_dma_transfer();          /* Try to send more data */
}

/* Send accumulated log message over command and control endpoint */
static void send_log_message_over_comms(void) {
	static volatile bool log_send_in_progress = false;

	if (!log_msg_rb_initialized) {
		lwrb_init(&log_msg_rb, log_msg_buffer, sizeof(log_msg_buffer));
		log_msg_rb_initialized = true;
	}

	/* Never transmit from interrupt context: comms_interface_send busy-waits
	 * on tx_flag, which only the USB OTG_HS IRQ sets — and that IRQ shares
	 * preemption priority 0 with several ISRs that printf, so the wait can
	 * never complete (2026-06-11 IMU streaming wedge). Keep the data
	 * buffered; the main loop flushes it via logging_pump(). */
	if (__get_IPSR() != 0U) {
		return;
	}

	/* Re-entry guard: comms_interface_send's failure paths printf, which
	 * lands back here on the newline. Recursing would clobber the shared
	 * txBuffer mid-build — keep the new data buffered instead. */
	if (log_send_in_progress) {
		return;
	}

	size_t linear_len = lwrb_get_linear_block_read_length(&log_msg_rb);
	if (linear_len == 0) {
		return; // Nothing to send
	}

	// Check if a transmission is already in progress
	// tx_flag == 1 means previous transmission is complete and we can send
	// tx_flag == 0 means a transmission is in progress, so queue the message
	// by keeping it in the buffer and returning (don't clear the buffer)
	if (tx_flag == 0) {
		// Transmission in progress, keep the message in the buffer (queue it)
		// The message will be sent the next time this function is called when tx_flag == 1
		// Note: If buffer is full, we'll need to handle that in add_char_to_log_buffer
		return;
	}

	// Prepare packet
	UartPacket log_packet;
	log_packet.id = 0; // ensure that every packet like this has a zero id to avoid confusion with the packet id
	log_packet.packet_type = OW_DATA;
	log_packet.command = OW_CMD_ECHO; // Use ECHO command for log messages
	log_packet.addr = 0;
	log_packet.reserved = 0;
	log_packet.data_len = (uint16_t)linear_len;
	log_packet.data = (uint8_t *)lwrb_get_linear_block_read_address(&log_msg_rb);
	log_packet.crc = 0; // CRC will be calculated by comms_interface_send

	// Send via comms interface using the shared txBuffer
	// Note: This may fail silently if USB is not initialized, which is acceptable
	// comms_interface_send handles packet building, CRC calculation, and transmission
	// For logging, we use a non-blocking approach - if it times out, we just skip
	log_send_in_progress = true;
	_Bool sent = comms_interface_send(&log_packet);
	log_send_in_progress = false;
	if (!sent) {
		// Transmission failed or timed out - keep message in buffer to retry later
		// Don't remove data from the buffer, so it can be retried on next call
		return;
	}

	// Remove only the bytes that were successfully sent
	lwrb_skip(&log_msg_rb, linear_len);
}

/* Flush USB log data that was buffered while in interrupt context.
 * Called from the main loop; no-op when there is nothing pending. */
void logging_pump(void) {
	if ((debug_flags & DEBUG_FLAG_USB_PRINTF) == 0u) {
		return;
	}
	send_log_message_over_comms();
}

/* Add character to log message buffer and send if newline encountered */
static void add_char_to_log_buffer(uint8_t ch) {
	if (!log_msg_rb_initialized) {
		lwrb_init(&log_msg_rb, log_msg_buffer, sizeof(log_msg_buffer));
		log_msg_rb_initialized = true;
	}

	// If buffer is full, drop some oldest data to make room.
	if (lwrb_get_free(&log_msg_rb) == 0) {
		size_t full = lwrb_get_full(&log_msg_rb);
		size_t drop = LOG_MSG_BUFFER_SIZE / 2;
		if (drop > full) {
			drop = full;
		}
		if (drop > 0) {
			lwrb_skip(&log_msg_rb, drop);
			log_msg_drop_marker_pending = true;
		}
	}

	// If we dropped data, inject a visible marker.
	if (log_msg_drop_marker_pending && lwrb_get_free(&log_msg_rb) > 0) {
		const uint8_t marker = '~';
		lwrb_write(&log_msg_rb, &marker, 1);
		log_msg_drop_marker_pending = false;
	}

	// Add character to buffer
	lwrb_write(&log_msg_rb, &ch, 1);

	// Send message when we encounter a newline
	// If transmission is not ready, the message stays in buffer (queued) and will be sent later
	// when send_log_message_over_comms is called again (on next newline or when buffer is full)
	if (ch == '\n') {
		send_log_message_over_comms();
		// After attempting to send, if there's still data in buffer (transmission was busy),
		// it will be sent on the next call when transmission becomes available
	}
}
