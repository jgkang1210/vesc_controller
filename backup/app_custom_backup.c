/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

#include "commands.h"

#include "conf_general.h"

#include "hw.h"
#include "packet.h"

#include "mc_interface.h"
#include "terminal.h"

 
#include "timeout.h"

#include <string.h>

// Settings
#define BAUDRATE					115200
#define PACKET_HANDLER				1
#define MOTOR_ID 					0x00

static volatile unsigned char instruction = 0;
static volatile unsigned char vel0 = 0;
static volatile unsigned char vel1 = 0;
static volatile unsigned char vel2 = 0;
static volatile unsigned char vel3 = 0;

static volatile int received_count = 0;
static volatile bool start_data = false;
static volatile bool receive_comp = false;

// Private variables
static volatile bool thread_is_running = false;
static volatile bool uart_is_running = false;

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);

// Thread for velocity control
static THD_FUNCTION(veolocity_control_thread, arg);
static THD_WORKING_AREA(veolocity_control_thread_wa, 8192); // 4kb stack for this thread

static SerialConfig uart_cfg = {
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

static void process_packet(unsigned char *data, unsigned int len) {
	commands_process_packet(data, len, app_uartcomm_send_packet);
}

static void send_packet(unsigned char *data, unsigned int len) {
	if (uart_is_running) {
		sdWrite(&HW_UART_DEV, data, len);
	}
}

void app_custom_start(void) {
	commands_printf("Starting the velocity control app");

	packet_init(send_packet, process_packet, PACKET_HANDLER);

	// Start the veolocity_control thread
	if (!thread_is_running) {
		chThdCreateStatic(veolocity_control_thread_wa, sizeof(veolocity_control_thread_wa), NORMALPRIO,
				veolocity_control_thread, NULL);
		thread_is_running = true;
	}

	sdStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);

	uart_is_running = true;
}

void app_custom_stop(void) {
	mc_interface_set_pid_speed(0);

	if (uart_is_running) {
		sdStop(&HW_UART_DEV);
		palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
		palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
		uart_is_running = false;
	}
}

static THD_FUNCTION(veolocity_control_thread, arg) {
	(void)arg;

	chRegSetThreadName("VELOCITY_CONTROL_CUSTOM");

	event_listener_t el;
	chEvtRegisterMaskWithFlags(&HW_UART_DEV.event, &el, EVENT_MASK(0), CHN_INPUT_AVAILABLE);

    for (;;) {
		// chEvtWaitAnyTimeout(ALL_EVENTS, ST2MS(10));

		bool rx = true;
		while (rx) {
			chThdSleepMilliseconds(8);

			rx = false;

			if (uart_is_running) {
				msg_t res = sdGetTimeout(&HW_UART_DEV, TIME_IMMEDIATE);
				if (res != MSG_TIMEOUT) {
					packet_process_byte(res, PACKET_HANDLER);
					rx = true;
				}

				volatile unsigned char c = (unsigned char)res;

				commands_printf("get %c", c);


				if (start_data == false) {
					if (c == 0x21) {
						start_data = true;
					}
					else {
						received_count = 0;
					}
				}
				else {
					if (received_count == 0) {
						if (c == 0xfe) {
							received_count++;
						}
						else {
							start_data = false;
							received_count = 0;
						}
					}
					else if (received_count == 1) {
						if (c == MOTOR_ID) {
							received_count++;
						}
						else {
							start_data = false;
							received_count = 0;
						}
					}
					else if (received_count == 2) {
						instruction = c;
						received_count++;
					}
					else if (received_count == 3) {
						vel0 = c;
						received_count++;
					}
					else if (received_count == 4) {
						vel1 = c;
						received_count++;
					}
					else if (received_count == 5) {
						vel2 = c;
						received_count++;
					}
					else if (received_count == 6) {
						vel3 = c;
						start_data = false;
						received_count = 0;
						receive_comp = true;
					}
				}

			}

			// flag up
			if (receive_comp == true) {
				// brake command
				if (instruction == 202) {
					mc_interface_brake_now();
				}
				// release command
				else if (instruction == 203) {
					mc_interface_release_motor();
				}
				// set the velocity controller
				// void mc_interface_set_pid_speed(float rpm);
				else if (instruction == 0xfe){
					float rpm = 0;
					unsigned char buff[4];

					buff[0] = vel0;
					buff[1] = vel1;
					buff[2] = vel2;
					buff[3] = vel3;

					memcpy(&rpm, buff, sizeof(float));

					commands_printf("111 %f", rpm);
					commands_printf("222 %f", rpm);
					// mc_interface_set_pid_speed(rpm);
				}

				// flag down
				receive_comp = false;
        	}


		}

		// Reset the timeout
		timeout_reset();
    }
}
