/*
 * uart_comms.c
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "uart_comms.h"
#include "if_commands.h"
#include "usbd_comms.h"
#include "usbd_def.h"
#include "usb_device.h"
#include "utils.h"
#include "logging.h"
#include "common.h"

#include <string.h>

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

// Private variables
extern uint8_t rxBuffer[COMMAND_MAX_SIZE];
extern uint8_t txBuffer[COMMAND_MAX_SIZE];
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream1;

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0; // start in idle state (0)
volatile uint8_t tx_flag = 1; // Start in idle state (1)
const uint32_t zero_val = 0;
static uint8_t cmd_data_buf[COMMAND_MAX_SIZE];

extern USBD_HandleTypeDef hUsbDeviceHS;

#define TX_TIMEOUT 500
#define RX_CMD_QUEUE_DEPTH 4U
#define RX_CMD_BUFFER_SIZE 512U

typedef struct {
	uint8_t data[RX_CMD_BUFFER_SIZE];
	uint16_t len;
} RxQueuedCommand;

static RxQueuedCommand rx_cmd_queue[RX_CMD_QUEUE_DEPTH];
static volatile uint8_t rx_cmd_q_head = 0;
static volatile uint8_t rx_cmd_q_tail = 0;
static volatile uint8_t rx_cmd_q_count = 0;

static uint8_t rx_work_buf[RX_CMD_BUFFER_SIZE];

__attribute__((unused))
static void print_uart_packet(const UartPacket *pkt)
{
    if (!pkt) {
        printf("UartPacket: NULL\r\n");
        return;
    }

    printf("UartPacket {\r\n");
    printf("  id           : 0x%04" PRIX16 " (%" PRIu16 ")\r\n", pkt->id, pkt->id);
    printf("  packet_type  : 0x%02X (%u)\r\n", pkt->packet_type, pkt->packet_type);
    printf("  command      : 0x%02X (%u)\r\n", pkt->command, pkt->command);
    printf("  addr         : 0x%02X (%u)\r\n", pkt->addr, pkt->addr);
    printf("  reserved     : 0x%02X\r\n", pkt->reserved);
    printf("  data_len     : %" PRIu16 "\r\n", pkt->data_len);

    // print data bytes if available
    if (pkt->data && pkt->data_len > 0) {
        printf("  data         : ");
        for (uint16_t i = 0; i < pkt->data_len; i++) {
            printf("%02X ", pkt->data[i]);
        }
        printf("\r\n");
    } else {
        printf("  data         : <none>\r\n");
    }

    printf("  crc          : 0x%04" PRIX16 " (%" PRIu16 ")\r\n", pkt->crc, pkt->crc);
    printf("}\r\n");
}

void ClearBuffer_DMA(void)
{
    // Clear using 32-bit writes (COMMAND_MAX_SIZE must be divisible by 4)
    HAL_DMA_Start(&hdma_memtomem_dma2_stream1,
                  (uint32_t)&zero_val,
                  (uint32_t)rxBuffer,
                  COMMAND_MAX_SIZE / 4);

    // Wait until transfer completes (blocking version)
    HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream1,
                            HAL_DMA_FULL_TRANSFER,
                            HAL_MAX_DELAY);
}

_Bool comms_interface_send(UartPacket *pResp) {
	static volatile _Bool send_in_progress = false;

	if (__get_IPSR() != 0U) {
		/* Interrupt context: the USB OTG_HS IRQ shares preemption priority
		 * 0 with several ISRs, so the tx_flag wait below could never
		 * complete — refuse instead of busy-waiting into a wedge. */
		return false;
	}
	if (send_in_progress) {
		/* Re-entered via a printf inside this function (the USB log path
		 * routes printf back here). No printf in this branch — that is
		 * exactly the recursion being refused. */
		return false;
	}
	send_in_progress = true;

	if(!tx_flag){
		printf("Comm tx not complete from last time");
		send_in_progress = false;
		return false;
	}

//    memset(txBuffer, 0, sizeof(txBuffer));
	int bufferIndex = 0;

	// Build the packet header
	txBuffer[bufferIndex++] = OW_START_BYTE;
	txBuffer[bufferIndex++] = pResp->id >> 8;
	txBuffer[bufferIndex++] = pResp->id & 0xFF;
	txBuffer[bufferIndex++] = pResp->packet_type;
	txBuffer[bufferIndex++] = pResp->command;
	txBuffer[bufferIndex++] = pResp->addr;
	txBuffer[bufferIndex++] = pResp->reserved;
	txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
	txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;

	// Check for possible buffer overflow (optional)
	uint32_t pkt_size = (bufferIndex + pResp->data_len + 4);
	if (pkt_size > sizeof(txBuffer)) {
		printf("Packet too large to send, len=%lu\r\n", pkt_size);
		// Handle error: packet too large for txBuffer
		send_in_progress = false;
		return false;
	}

	// Add data payload if any
	if (pResp->data_len > 0) {
		memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
		bufferIndex += pResp->data_len;
	}

	// Compute CRC over the packet from index 1 for (pResp->data_len + 8) bytes
	uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 8);
	txBuffer[bufferIndex++] = crc >> 8;
	txBuffer[bufferIndex++] = crc & 0xFF;

	// Add the end byte
	txBuffer[bufferIndex++] = OW_END_BYTE;

	// Initiate transmission via USB CDC
	tx_flag = 0;  // Set the flag before starting transmission
	uint8_t tx_status = USBD_COMMS_SendData(&hUsbDeviceHS, txBuffer, bufferIndex, 0);
	if (tx_status != USBD_OK) {
		// Transmission not started (e.g., endpoint busy); don't block on tx_flag
		printf("COMM USB TX failed: id=0x%04X cmd=0x%02X type=0x%02X len=%d dev_state=0x%02X\r\n",
			   pResp->id, pResp->command, pResp->packet_type, bufferIndex, hUsbDeviceHS.dev_state);
		if (tx_status == USBD_BUSY) {
			printf("COMM USB TX failed: endpoint busy\r\n");
		} else if (tx_status == USBD_FAIL) {
			printf("COMM USB TX failed: USBD_FAIL\r\n");
		} else {
			printf("COMM USB TX failed: status=0x%02X\r\n", tx_status);
		}
		tx_flag = 1; // reset to idle on failure
		USB_NotifyTxFailure();
		send_in_progress = false;
		return false;
	}

	if(pResp->command == OW_CAMERA_GET_HISTOGRAM && verbose_on)
	{
		printf("F:%d C: 0x%02X V: 0x%02X S:%d\r\n", pResp->id, pResp->addr, pResp->reserved, bufferIndex);
	}

	// Wait for the transmit complete flag with a timeout to avoid infinite loop.
	uint32_t start_time = get_timestamp_ms();

	while (!tx_flag) {
		if ((get_timestamp_ms() - start_time) >= TX_TIMEOUT) {
			// Timeout handling: Log error and break out or reset the flag.
			// printf before releasing send_in_progress so the message only
			// buffers instead of re-entering this function.
			printf("COMM USB TX Timeout\r\n");
			tx_flag = 1; // reset to idle on timeout
			USB_NotifyTxFailure();
			send_in_progress = false;
			return false;
		}
	}
	send_in_progress = false;
	return true;
}

void comms_host_start(void) {

	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;

	USBD_COMMS_FlushRxBuffer();

	rx_flag = 0;
	tx_flag = 1;
	rx_cmd_q_head = 0;
	rx_cmd_q_tail = 0;
	rx_cmd_q_count = 0;

}

// This is the FreeRTOS task
void comms_host_check_received(void) {
	UartPacket cmd;
	UartPacket resp = {0};   /* zero-init: error/NAK paths don't set every field (e.g. resp.command); avoids -Os uninitialized read */
	uint16_t calculated_crc;
	uint16_t rx_len;

	__disable_irq();
	if (rx_cmd_q_count == 0) {
		__enable_irq();
		return;
	}
	rx_len = rx_cmd_queue[rx_cmd_q_tail].len;
	memcpy(rx_work_buf, rx_cmd_queue[rx_cmd_q_tail].data, rx_len);
	rx_cmd_q_tail = (uint8_t)((rx_cmd_q_tail + 1U) % RX_CMD_QUEUE_DEPTH);
	rx_cmd_q_count--;
	rx_flag = (rx_cmd_q_count > 0U) ? 1U : 0U;
	__enable_irq();

	int bufferIndex = 0;

	if (rx_len < 12U) {
		resp.id = 0xFFFF;
		resp.data_len = 0;
		resp.addr = 0;
		resp.reserved = 0;
		resp.packet_type = OW_ERROR;
		goto NextDataPacket;
	}

	if (rx_work_buf[bufferIndex++] != OW_START_BYTE) {
		// Send NACK doesn't have the correct start byte
		resp.id = 0xFFFF;
		resp.data_len = 0;
		resp.addr = 0;
		resp.reserved = 0;
		resp.packet_type = OW_NAK;
		goto NextDataPacket;
	}

	cmd.id = (rx_work_buf[bufferIndex] << 8 | (rx_work_buf[bufferIndex + 1] & 0xFF));
	bufferIndex += 2;
	if ((logging_get_debug_flags() & DEBUG_FLAG_COMM_VERBOSE) != 0u) {
		printf("0x%04X\r\n", cmd.id);
	}
	cmd.packet_type = rx_work_buf[bufferIndex++];
	cmd.command = rx_work_buf[bufferIndex++];
	cmd.addr = rx_work_buf[bufferIndex++];
	cmd.reserved = rx_work_buf[bufferIndex++];

	// Extract payload length
	cmd.data_len = (rx_work_buf[bufferIndex] << 8
			| (rx_work_buf[bufferIndex + 1] & 0xFF));
	bufferIndex += 2;

	// Check if data length is valid
	if ((cmd.data_len > (uint16_t)(rx_len - bufferIndex))
			&& rx_work_buf[rx_len - 1] != OW_END_BYTE) {
		// Send NACK response due to no end byte
		// data can exceed buffersize but every buffer must have a start and end packet
		// command that will send more data than one buffer will follow with data packets to complete the request
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = 0;
		resp.data_len = 0;
		resp.packet_type = OW_ERROR;
		goto NextDataPacket;
	}

	// Copy data payload out of rxBuffer so rxBuffer is no longer aliased during processing
	uint16_t copy_len = (cmd.data_len > (uint16_t)(rx_len - bufferIndex))
	                    ? (uint16_t)(rx_len - 3U - bufferIndex)
	                    : cmd.data_len;
	memcpy(cmd_data_buf, &rx_work_buf[bufferIndex], copy_len);
	cmd.data = cmd_data_buf;
	if (cmd.data_len > (uint16_t)(rx_len - bufferIndex)) {
		bufferIndex = rx_len - 3U; // [3 bytes from the end should be the crc for a continuation packet]
	} else {
		bufferIndex += cmd.data_len; // move pointer to end of data
	}

	// Extract received CRC
	cmd.crc = (rx_work_buf[bufferIndex] << 8 | (rx_work_buf[bufferIndex + 1] & 0xFF));
	bufferIndex += 2;

	// Calculate CRC for received data

	if (cmd.data_len > copy_len) {
		calculated_crc = util_crc16(&rx_work_buf[1], rx_len - 3U);
	} else {
		calculated_crc = util_crc16(&rx_work_buf[1], cmd.data_len + 8);
	}

	// Check CRC
	if (cmd.crc != calculated_crc) {
		// Send NACK response due to bad CRC
		resp.id = cmd.id;
		resp.addr = 0;
		resp.reserved = OW_BAD_CRC;
		resp.data_len = 0;
		resp.packet_type = OW_ERROR;
		goto NextDataPacket;
	}

	// Check end byte
	if (rx_work_buf[bufferIndex++] != OW_END_BYTE) {
		resp.id = cmd.id;
		resp.data_len = 0;
		resp.addr = 0;
		resp.reserved = 0;
		resp.packet_type = OW_ERROR;
		goto NextDataPacket;
	}

	resp = process_if_command(cmd);

NextDataPacket:
    const bool success = comms_interface_send(&resp);
	// printf("[RESP] ID:0x%04X Cmd:0x%02X Type:0x%02X -> Resp:0x%02X Len:%d\r\n",
	// 	   cmd.id, cmd.command, cmd.packet_type, resp.packet_type, resp.data_len);
	ptrReceive = 0;
	if ((logging_get_debug_flags() & DEBUG_FLAG_COMM_VERBOSE) != 0u) {
		if(success){
			printf(".\r\n");
		}
		else {
			printf("!\r\n");
		}
	}

}


// Callback functions
void USBD_COMMS_RxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum) {
	static volatile uint32_t rx_overrun_count = 0;
	uint16_t copy_len;
	uint8_t q_head;

	if (rx_cmd_q_count >= RX_CMD_QUEUE_DEPTH) {
		rx_overrun_count++;
		printf("COMM RX QUEUE FULL: count=%lu len=%lu\r\n",
			   (unsigned long)rx_overrun_count,
			   (unsigned long)Len);
		return;
	}

	copy_len = (Len > RX_CMD_BUFFER_SIZE) ? RX_CMD_BUFFER_SIZE : (uint16_t)Len;
	q_head = rx_cmd_q_head;
	memcpy(rx_cmd_queue[q_head].data, Buf, copy_len);
	rx_cmd_queue[q_head].len = copy_len;
	rx_cmd_q_head = (uint8_t)((rx_cmd_q_head + 1U) % RX_CMD_QUEUE_DEPTH);
	rx_cmd_q_count++;
	rx_flag = 1;

	if (Len > RX_CMD_BUFFER_SIZE) {
		printf("COMM RX truncated: len=%lu max=%u\r\n",
			   (unsigned long)Len,
			   (unsigned int)RX_CMD_BUFFER_SIZE);
	}
}

void USBD_COMMS_TxCpltCallback(uint8_t *Buf, uint32_t Len, uint8_t epnum) {
	tx_flag = 1;
	USB_NotifyTxSuccess();
}

