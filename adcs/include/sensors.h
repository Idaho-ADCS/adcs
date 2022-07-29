/**
 * @defgroup   SENSORS sensors.cpp
 *
 * @brief      This file implements sensor initialization functions for main.cpp, data structures, and functions for reading from the sensors in RTOS.
 *
 * @author     Garrett Wells, Parker Piedmont
 * @date       2022
 */
#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "global_definitions.h"
#include "FreeRTOS_SAMD51.h"
#include "ADCSPhotodiodeArray.h"
#include "ICM_20948.h"
#include "INA209.h"

#define NUM_IMUS 1
#define INA 1
#define pds 1

// SENSOR VARIABLES DEFINED IN `sensors.cpp` //////////////////////////////////////
extern INA209 ina209;
extern ICM_20948_I2C IMU2;
extern ICM_20948_I2C IMU1;
extern ADCSPhotodiodeArray sunSensors;
// RTOS VARIABLES DEFINED IN `sensors.cpp` ///////////////////////////////////////
extern QueueHandle_t IMUq;
extern QueueHandle_t INAq;
extern QueueHandle_t PDq;
extern SemaphoreHandle_t IMUsemphr;
extern SemaphoreHandle_t INAsemphr;
extern SemaphoreHandle_t PDsemphr;
//////////////////////////////////////////////////////////////////////////////////

/* DATA TYPES =============================================================== */

// magnetometer and gyroscope data from IMU
typedef struct
{
	float magX;
	float magY;
	float magZ;
	float gyrX;
	float gyrY;
	float gyrZ;
} IMUdata;

// voltage and current from INA209
typedef struct
{
	float voltage;
	int current;
} INAdata;

// photodiode data
typedef union
{
	float data[6];

	struct
	{
		float x_pos;
		float x_neg;
		float y_pos;
		float y_neg;
		float z_pos;
		float z_neg;
	};
} PDdata;

typedef union
{
	int data[6];

	struct
	{
		int x_pos;
		int x_neg;
		int y_pos;
		int y_neg;
		int z_pos;
		int z_neg;
	};
} PDdata_int;

/* HARDWARE INIT FUNCTIONS ================================================== */

void initIMU(void);
void initINA(void);
void initSunSensors(void);

/* SENSOR READING FUNCTIONS ================================================= */

INAdata readINA(void);
PDdata readPD(void);
PDdata_int read_filtered_PD(void);
int simple_PD_filter(int channel);

/* SENSOR RTOS TASKS ======================================================== */

void readIMU(void *pvParameters);
void readINA_rtos(void *pvParameters);

/* PRINTING FUNCTIONS ======================================================= */

// "helper" functions from IMU demo code that print sensor data to serial
// monitor over USB
void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
void printScaledAGMT(ICM_20948_I2C *sensor);

#endif