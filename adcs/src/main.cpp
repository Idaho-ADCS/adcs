// Our own headers
#include "comm.h"
#include "sensors.h"
#include "supportFunctions.h"
#include "commandFunctions.h"

// Arduino library headers
#include "DRV_10970.h"
#include "INA209.h"
#include "ICM_20948.h"
#include <FreeRTOS_SAMD51.h>

// Standard C/C++ library headers
#include <stdint.h>

// if defined, enables debug print statements over USB to the serial monitor
#define DEBUG

/* NON-RTOS GLOBAL VARIABLES ================================================ */

/**
 * @brief
 * IMU objects, attached to IMUs. Used to read data from IMUs.
 */
ICM_20948_I2C IMU1;
#ifdef TWO_IMUS
ICM_20948_I2C IMU2;
#endif

/* RTOS TASK GLOBAL VARIABLES =============================================== */

/**
 * @brief
 * FreeRTOS queue that stores the current mode of the ADCS.
 * Possible values:
 *   MODE_STANDBY (0)
 *   MODE_TEST    (1)
 */
QueueHandle_t modeQ;

/* RTOS TASK DECLARATIONS =================================================== */

static void readUART(void *pvParameters);
static void writeUART(void *pvParameters);

/* "MAIN" =================================================================== */

/**
 * @brief
 * Since main is already defined by the Arduino framework, we will use the
 * setup function as if it were main. Since setup runs only once, it
 * essentially behaves the same as main.
 */
void setup()
{
	/**
	 * @brief
	 * INA209 object
	 */
	// INA209* ina209;
	INA209 ina209(0x80);

	/**
	 * @brief
	 * DRV10970 object, connected to the motor driver of the flywheel
	 */
	DRV10970* DRV;

	/**
	 * Create a counting semaphore with a maximum value of 1 and an initial
	 * value of 0. Starts ADCS in standby mode.
	 */
	modeQ = xQueueCreate(1, sizeof(uint8_t));
	uint8_t mode = 0;
	xQueueSend(modeQ, (void*)&mode, (TickType_t)0);

	// enable LED
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, HIGH);

#ifdef DEBUG
    /**
     * Initialize USB connection to computer. Used to print debug messages.
     * Baud rate: 115200
     * Data bits: 8
     * Parity: none
     */
    SERCOM_USB.begin(115200);
    while (!SERCOM_USB);  // wait for initialization to complete
    SERCOM_USB.write("USB interface initialized\r\n");
#endif

    /**
     * Initialize UART connection to satellite
     * Baud rate: 115200
     * Data bits: 8
     * Parity: odd (1 bit)
     */
    SERCOM_UART.begin(115200, SERIAL_8O1);
    while (!SERCOM_UART);  // wait for initialization to complete
#ifdef DEBUG
    SERCOM_USB.write("UART interface initialized\r\n");
#endif

    /**
     * Initialize I2C network
     * Clock: 400 kHz
     */
    SERCOM_I2C.begin();
    SERCOM_I2C.setClock(400000);

	/**
	 * Initialize first IMU
	 * Address: 0x68
	 */
    IMU1.begin(SERCOM_I2C, AD0_VAL);
    while (IMU1.status != ICM_20948_Stat_Ok);  // wait for initialization to
                                               // complete
#ifdef DEBUG
    SERCOM_USB.write("IMU1 initialized\r\n");
#endif

#ifdef TWO_IMUS
	/**
	 * Initialize second IMU
	 * Address: 0x69
	 */
    IMU2.begin(SERCOM_I2C, (~AD0_VAL)&1);
    while (IMU2.status != ICM_20948_Stat_Ok);  // wait for initialization to
                                               // complete
	#ifdef DEBUG
    SERCOM_USB.write("IMU2 initialized\r\n");
	#endif
#endif

    // TODO init INA209 with real values, defaults are for 32V system
    ina209.writeCfgReg(14751); // default
    ina209.writeCal(4096);
    
#ifdef DEBUG
    SERCOM_USB.write("INA209 initialized\r\n");
#endif

    // initialization completed, notify satellite
	ADCSdata data_packet;
	clearADCSdata(&data_packet);
    data_packet.status = STATUS_HELLO;
    // TODO: compute CRC
    SERCOM_UART.write(data_packet.data, PACKET_LEN);

    // instantiate tasks and start scheduler
    xTaskCreate(readUART, "Read UART", 2048, NULL, 1, NULL);
    xTaskCreate(writeUART, "Write UART", 2048, NULL, 1, NULL);
    // TODO: schedule task for INA209 read

#ifdef DEBUG
    SERCOM_USB.write("Tasks created\r\n");
#endif

    vTaskStartScheduler();

    // should never be reached if everything goes right
    while (1);
}

/* RTOS TASK DEFINITIONS ==================================================== */

/**
 * @brief
 * Polls the UART module for data. Processes data one byte at a time if the
 * module reports that data is ready to be received.
 * 
 * @param[in] pvParameters  Unused but required by FreeRTOS. Program will not
 * compile without this parameter. When a task is instantiated from this
 * function, a set of initialization arguments or NULL is passed in as
 * pvParameters, so pvParameters must be declared even if it is not used.
 * 
 * @return None
 * 
 * TODO: Remove polling and invoke this task using an interrupt instead.
 */
static void readUART(void *pvParameters)
{
	TEScommand cmd_packet;
	uint8_t mode;

    uint8_t bytes_received = 0;  // number of consecutive bytes received from
                                 // satellite - used as index for cmd packet
                                 // char array
#ifdef DEBUG
    char cmd_str[8];  // used to print command value to serial monitor
#endif

    while (1)
    {
        if (SERCOM_UART.available())  // at least one byte is in the UART
        {							  // receive buffer

            // copy one byte out of UART receive buffer
            cmd_packet.data[bytes_received] = SERCOM_UART.read();
            bytes_received++;

            if (bytes_received >= COMMAND_LEN)  // full command packet received
            {
                // TODO: verify CRC

                if (cmd_packet.command == COMMAND_TEST)
                {
                    mode = MODE_TEST;
                }

                if (cmd_packet.command == COMMAND_STANDBY)
                {
                    mode = MODE_STANDBY;
                }

				xQueueOverwrite(modeQ, (void*)&mode);  // enter specified mode

#ifdef DEBUG
                // convert int to string for USB monitoring
                sprintf(cmd_str, "0x%02x", cmd_packet.command);

                // print command value to USB
                SERCOM_USB.print("Command received: ");
                SERCOM_USB.print(cmd_str);
                SERCOM_USB.print("\r\n");

                if (cmd_packet.command == COMMAND_TEST)
                {
                    SERCOM_USB.print("Entering test mode\r\n");
                }

                if (cmd_packet.command == COMMAND_STANDBY)
                {
                    SERCOM_USB.print("Entering standby mode\r\n");
                }
#endif

                // reset index counter to zero for next command
                bytes_received = 0;
            }
        }

		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief
 * Reads magnetometer and gyroscope values from IMU and writes them to UART
 * every 0.5 seconds while ADCS is in test mode.
 * 
 * @param[in] pvParameters  Unused but required by FreeRTOS. Program will not
 * compile without this parameter. When a task is instantiated from this
 * function, a set of initialization arguments or NULL is passed in as
 * pvParameters, so pvParameters must be declared even if it is not used.
 * 
 * @return None
 */
static void writeUART(void *pvParameters)
{
	UBaseType_t mode;
	ADCSdata data_packet;    

    while (1)
    {
		xQueuePeek(modeQ, (void*)&mode, (TickType_t)0);

        if (mode == MODE_TEST)
        {
			data_packet.status = STATUS_OK;

			// use static dummy values for voltage and current until we figure
			// out how to use the INA
			data_packet.voltage = 6;
			data_packet.current = 500 / 10;

			// use static dummy value for motor speed until frequency
			// measurements work
			data_packet.speed = floatToFixed(1.0);

			readIMU(&data_packet);

			// TODO: compute CRC

			SERCOM_UART.write(data_packet.data, PACKET_LEN);  // send to TES
#ifdef DEBUG
			SERCOM_USB.write("Wrote to UART\r\n");
			// printScaledAGMT(&IMU1);
#endif

			clearADCSdata(&data_packet);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    // do nothing
}