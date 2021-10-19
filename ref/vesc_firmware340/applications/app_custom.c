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
#include "commands.h"
#include "conf_general.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "packet.h"

#include <string.h>

#define MOTOR_ID 0x06

	// Private variables

static volatile unsigned char instruction = 0;
static volatile unsigned char pos0 = 0;
static volatile unsigned char pos1 = 0;
static volatile unsigned char pos2 = 0;
static volatile unsigned char pos3 = 0;

static volatile float pppp = 0;

static volatile int received_count = 0;
static volatile bool start_data = false;
static volatile bool receive_comp = false;
// Example thread
static THD_FUNCTION(example_thread, arg);
static THD_WORKING_AREA(example_thread_wa, 4096); // 2kb stack for this thread

												  /*
												  * This callback is invoked when a transmission buffer has been completely
												  * read by the driver.
												  */
static void txend1(UARTDriver *uartp) { (void)uartp; }

/*
* This callback is invoked when a transmission has physically completed.
*/
static void txend2(UARTDriver *uartp) { (void)uartp; }

/*
* This callback is invoked on a receive error, the errors mask is passed
* as parameter.
*/
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

/*
* This callback is invoked when a character is received but the application
* was not ready to receive it, the character is passed as parameter.
*/
static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;
//	pppp = c;
//commands_printf("%d\r\n", c); 
	// pos_data2 = c;
	// receive_comp = true;

	if (start_data == false) {
		if (c == 0xff) {
			start_data = true;
		}
	}
	else {
		if (received_count == 0) {
			if (c == 0xff) {
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
			pos0 = c;
			received_count++;
		}
		else if (received_count == 4) {
			pos1 = c;
			received_count++;
		}
		else if (received_count == 5) {
			pos2 = c;
			received_count++;
		}
		else if (received_count == 6) {
			pos3 = c;
			start_data = false;
			received_count = 0;
			receive_comp = true;
		}
	}
}

/*
* This callback is invoked when a receive buffer has been completely written.
*/
static void rxend(UARTDriver *uartp) { (void)uartp; }

/*
* UART driver configuration structure.
*/
static UARTConfig uart_cfg = { txend1, txend2, rxend,           rxchar, rxerr,
115200, 0,      USART_CR2_LINEN, 0 };

void app_custom_start(void) {
	// Start the example thread
	chThdCreateStatic(example_thread_wa, sizeof(example_thread_wa), NORMALPRIO,
		example_thread, NULL);
}

static THD_FUNCTION(example_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_CUSTOM");

	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN,
		PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST |
		PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN,
		PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST |
		PAL_STM32_PUDR_PULLUP);

	for (;;) {
//commands_printf("instruction : %f\r\n", pppp); 
		if (receive_comp == true) {
			// If the button is pressed, run the motor with speed control
			// proportional to the POT position with a speed between 0 ERPM
			// and 10000 ERPM
			//commands_printf("instruction : %f\r\n", pppp); 
				if (instruction == 202) {
					mc_interface_brake_now();
				}
				else if (instruction == 203) {
					mc_interface_release_motor();
				}
				else {
					union {
					float real;
					uint32_t base;
				} u_Pos;
				u_Pos.base = 0;
				u_Pos.base |= ((uint32_t)(pos0)) << (8 * 0);
				u_Pos.base |= ((uint32_t)(pos1)) << (8 * 1);
				u_Pos.base |= ((uint32_t)(pos2)) << (8 * 2);
				u_Pos.base |= ((uint32_t)(pos3)) << (8 * 3);
				//commands_printf("set pid pos : %f\r\n", u_Pos.real);
				mc_interface_set_pid_pos(u_Pos.real);
				}
			receive_comp = false;
		}
		else {
		}

		// Run this loop at 500Hz
		chThdSleepMilliseconds(2);

		// Reset the timeout
		timeout_reset();
	}
}
