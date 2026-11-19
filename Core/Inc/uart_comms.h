/*
 * uart_comms.h
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_UART_COMMS_H_
#define INC_UART_COMMS_H_

#include "main.h"  // This should contain your HAL includes and other basic includes.
#include "common.h"
#include <stdio.h>
#include <stdbool.h>


void comms_host_check_received(void);
void comms_host_start(void);
_Bool comms_interface_send(const UartPacket* pResp);


#endif /* INC_UART_COMMS_H_ */
