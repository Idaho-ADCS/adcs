/**
 * @defgroup   MAIN main.cpp
 *
 * @brief      This file implements main, initializes hardware, RTOS, and communication buses.
 *
 * @author     Garrett Wells, Parker Piedmont, Kristie Olds
 * @date       2022
 */
#include "global_definitions.h"
#include "actuators.h"
#include "sensors.h"
#include "rtos_tasks.h"

// Standard C/C++ library headers
#include <stdint.h>

/* NON-RTOS GLOBAL VARIABLES ================================================ */



/* RTOS GLOBAL VARIABLES ==================================================== */

/**
 * @brief      	FreeRTOS queue that stores the current mode of the ADCS.
 * Possible values:
 *   MODE_STANDBY (0)
 *   MODE_HEARTBEAT    (1)
 */
QueueHandle_t modeQ;

/* HELPER FUNCTIONS ========================================================= */

void blinkLED(unsigned int num);

/* "MAIN" =================================================================== */

/**
 * @brief      Since main is already defined by the Arduino framework, we will use the
 * setup function as if it were main. Since setup runs only once, it
 * essentially behaves the same as main.
 */
void setup()
{
	delay(1000);

	ADCSdata data_packet;

	// Create a counting semaphore with a maximum value of 1 and an initial
	// value of 0. Starts ADCS in standby mode.
	modeQ = xQueueCreate(1, sizeof(uint8_t));
	uint8_t mode = CMD_STANDBY;
	xQueueSend(modeQ, (void *)&mode, (TickType_t)0);

	// enable LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);


	#if DEBUG
		initUSB();
		// data_packet.setStatus(0x01);
		// blinkLED(1);
	#endif

	initUART();
	// data_packet.setStatus(0x02);
	// blinkLED(2);

	#if NUM_IMUS > 0
		initI2C();
		// data_packet.setStatus(0x03);
		// blinkLED(3);

		initIMU();
		// data_packet.setStatus(0x04);
		// blinkLED(4);
	#endif

	#if INA
		initINA();
		// data_packet.setStatus(0x05);
		// blinkLED(5);
	#endif

	initSunSensors();
	initFlyWhl();
	initMtx();

	pinMode(9, OUTPUT);
	digitalWrite(9, HIGH); // set the direction pin HIGH??
	#if DEBUG
		SERCOM_USB.print("[system init]\tMotor direction set high\r\n");
	#endif
	// data_packet.setStatus(0x06);
	// blinkLED(6);

	pinMode(10, OUTPUT);
	analogWrite(10, 0); // set the PWM pin to 0%
	#if DEBUG
		SERCOM_USB.print("[system init]\tPWM set to 0%\r\n");
	#endif
	// data_packet.setStatus(0x07);
	// blinkLED(7);

	// instantiate tasks and start scheduler
	xTaskCreate(receiveCommand, "Read UART", 256, NULL, 3, NULL);
	#if DEBUG
		SERCOM_USB.print("[rtos]\t\tTask receiveCommand created\r\n");
	#endif

	xTaskCreate(heartbeat, "Write UART", 256, NULL, 2, NULL);
	#if DEBUG
		SERCOM_USB.print("[rtos]\t\tTask heartbeat created\r\n");
	#endif

	create_test_tasks();
	// data_packet.setStatus(0x08);
	// blinkLED(8);

	// initialization completed, notify satellite
	// ADCSdata data_packet;
	data_packet.setStatus(STATUS_HELLO);
	data_packet.send();
	// blinkLED(9);

	// delay(10);
	// digitalWrite(LED_BUILTIN, HIGH);

	#if DEBUG
		SERCOM_USB.print("[rtos]\t\tStarting task scheduler\r\n");
	#endif
	vTaskStartScheduler();

	// should never be reached if everything goes right
	while (1)
	{
		#if DEBUG
			SERCOM_USB.println("ERROR");
		#endif
		data_packet.setStatus(STATUS_ADCS_ERROR);
		data_packet.send();
		delay(1000);
	}
}

/**
 * @brief
 * Does nothing. Since we are using FreeRTOS, we will not use Arduino's loop
 * function. However, the project will fail to compile if loop is not defined.
 * Therefore, we define loop to do nothing.
 *
 * TODO: Eliminate this function entirely? Even though it does nothing, it will
 * still likely be called and consume clock cycles.
 */
void loop()
{
	// do nothing or enter low power mode
	// blinkLED(1);
}

/**
 * @brief      Default exception handler which is called internally by the microcontroller
 *
 * @param[in]  ulCause   The cause
 * @param[in]  ulStatus  The status
 */
void _general_exception_handler(unsigned long ulCause, unsigned long ulStatus)
{
	/* This overrides the definition provided by the kernel.  Other exceptions
	should be handled here. */

	ADCSdata error_msg;
	error_msg.setStatus(STATUS_ADCS_ERROR);

	while (1)
	{
		#if DEBUG
			SERCOM_USB.println("ERROR");
		#endif
		error_msg.send();
		digitalWrite(LED_BUILTIN, HIGH);
		vNopDelayMS(1000);
		digitalWrite(LED_BUILTIN, LOW);
		vNopDelayMS(5000);
	}
}

/**
 * @brief      Blink builtin LED some number of times, with quarter second delay
 *
 * @param[in]  num   The number of blinks
 */
void blinkLED(unsigned int num)
{
	digitalWrite(LED_BUILTIN, LOW);
	delay(500);

	for (unsigned int i = 0; i < num; i++)
	{
		digitalWrite(LED_BUILTIN, HIGH);
		delay(250);
		digitalWrite(LED_BUILTIN, LOW);
		delay(250);
	}

	delay(500);
}