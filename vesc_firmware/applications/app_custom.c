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

#ifdef APP_CUSTOM_TO_USE
#include APP_CUSTOM_TO_USE

#include "app.h"
#include "ch.h"
#include "commands.h"
#include "conf_general.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "packet.h"

#include <string.h>

#define MOTOR_ID 0x00

static volatile unsigned char instruction = 0;
static volatile unsigned char vel0 = 0;
static volatile unsigned char vel1 = 0;
static volatile unsigned char vel2 = 0;
static volatile unsigned char vel3 = 0;

static volatile int received_count = 0;
static volatile bool start_data = false;
static volatile bool receive_comp = false;

// Thread for velocity control
static THD_FUNCTION(veolocity_control_thread, arg);
static THD_WORKING_AREA(veolocity_control_thread_wa, 4096); // 4kb stack for this thread

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
* This callback is invoked when a character is received but the application
* was not ready to receive it, the character is passed as parameter.
*/
static void rxchar(UARTDriver *uartp, uint16_t c);

/*
* This callback is invoked when a receive buffer has been completely written.
*/
static void rxend(UARTDriver *uartp) { (void)uartp; }

/*
* This callback is invoked on a receive error, the errors mask is passed
* as parameter.
*/
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

static UARTConfig uart_cfg = { txend1, txend2, rxend, rxchar, rxerr,
                                115200, 0, SART_CR2_LINEN, 0 };

static void rxchar(UARTDriver *uartp, uint16_t c){
    (void)uartp;

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

void app_custom_start(void) {
	// Start the veolocity_control thread
	chThdCreateStatic(veolocity_control_thread_wa, sizeof(veolocity_control_thread_wa), NORMALPRIO,
		veolocity_control_thread, NULL);
}

static THD_FUNCTION(veolocity_control_thread, arg) {
	(void)arg;

	chRegSetThreadName("VELOCITY_CONTROL_CUSTOM");

    uartStart(&HW_UART_DEV, &uart_cfg);
    palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN,
		PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST |
		PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN,
		PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST |
		PAL_STM32_PUDR_PULLUP);

    for (;;) {
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
            else{
                float rpm = 0;
                unsigned char buff[4];

                buff[0] = vel0;
                buff[1] = vel1;
                buff[2] = vel2;
                buff[3] = vel3;

                static_assert(sizeof(float) == 4);

                memcpy(&rpm, &buff[4], sizeof(float));

                mc_interface_set_pid_speed(rpm);
            }
        }
        // Run this loop at 500Hz
		chThdSleepMilliseconds(2);

		// Reset the timeout
		timeout_reset();
    }
}

#endif
