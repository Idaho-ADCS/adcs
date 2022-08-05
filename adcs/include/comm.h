/**
 * @defgroup   COMM comm.cpp
 *
 * @brief      This file implements communications data structures, custom data types, data conversion functions, commands, and status codes for the satellite and ADCS communications.
 *
 * @author     Parker Piedmont
 * @date       2022
 */
#ifndef __COMM_H__
#define __COMM_H__

#include "global_definitions.h"
#include "sensors.h"
#include <CRC16.h>
#include <Wire.h>
#include <stdint.h>

// packet sizes in bytes
#define COMMAND_LEN 4
#define PACKET_LEN 26

/**
 * @brief      Commands that the ADCS should expect to receive from the satellite
 */
enum Command : uint8_t
{
	CMD_DESATURATE = 0x00, // bring everything to a stop, maybe turn off?
	CMD_STANDBY = 0xc0,
	CMD_HEARTBEAT = 0xa0, // transmit heartbeat signal regularly

	CMD_TST_BASIC_MOTION = 0xa1,	// test how much force needed to rotate
	CMD_TST_BASIC_AD = 0xa2,		// test attitude determination
	CMD_TST_BASIC_AC = 0xa3,		// test attitude control
	CMD_TST_SIMPLE_DETUMBLE = 0xa4, // test simplistic detumble
	CMD_TST_SIMPLE_ORIENT = 0xa5,	// test simplistic orientation
	CMD_TST_PHOTODIODES = 0xa6,	// test photodiodes
	CMD_TST_BLDC = 0xa7,	// test functionality of BLDC
	CMD_TST_MTX = 0xa8,	// test functionality of magnetorquers



	CMD_ORIENT_DEFAULT = 0x80, // should be orienting to something like X+
	CMD_ORIENT_X_POS = 0xe0,
	CMD_ORIENT_Y_POS = 0xe1,
	CMD_ORIENT_X_NEG = 0xe2,
	CMD_ORIENT_Y_NEG = 0xe3
};

/**
 * @brief      ADCS system status codes to send to the satellite
 */
enum Status : uint8_t
{
	STATUS_OK = 0xaa,		  // "Heartbeat"
	STATUS_HELLO = 0xaf,	  // Sent upon system init
	STATUS_ADCS_ERROR = 0xf0, // Sent upon runtime error
	STATUS_COMM_ERROR = 0x99, // Sent upon invalid communication
	STATUS_FUDGED = 0x00,	  // Data is not real, just test output

	STATUS_TEST_START = 0xb0, // starting test
	STATUS_TEST_END = 0xb1,	  // test finished
};

/**
 * @brief
 * Defines a fixed-point data type that is one byte wide. Five bits are reserved
 * for the integer part and 3 bits are reserved for the fraction part. While in
 * fixed-point form, the data cannot be read - it must be converted back to
 * floating-point first. Functions that convert float to fixed and back are
 * declared below.
 */
typedef int8_t fixed5_3_t;

/**
 * @brief
 */
class TEScommand
{
private:
	// Stores data from TES as a union of an array and integers
	union
	{
		// Data can be accessed as a single array - used when receiving bytes
		uint8_t _data[COMMAND_LEN];

		struct
		{
			// Data can be accessed as fields - used to extract command
			uint16_t _command;
			uint16_t _crc;
		};
	};

	// Counts the number of bytes received to see if the packet is full
	uint8_t _bytes_received;

	// Flag that indicates when the packet is full
	bool _full;

public:
	/**
	 *
	 */
	TEScommand();

	/**
	 *
	 */
	void addByte(uint8_t b);
	void loadBytes(uint8_t *bytes);
	bool isFull();
	uint8_t getCommand();
	bool checkCRC();
	void clear();
};

class ADCSdata
{
private:
	union
	{
		// Data can be accessed as a single array - used to send via UART
		uint8_t _data[PACKET_LEN];

		struct
		{
			// Data can be accessed as fields - used to build packet
			uint16_t _status; //2
			fixed5_3_t _voltage; //1
			int16_t _current; //2
			uint8_t _speed; //1
			int8_t _magX; //1
			int8_t _magY; //1
			int8_t _magZ; //1
			fixed5_3_t _gyroX; //1
			fixed5_3_t _gyroY; //1
			fixed5_3_t _gyroZ; //1
			uint16_t _pd_xpos; //2
			uint16_t _pd_xneg; //2
			uint16_t _pd_ypos; //2
			uint16_t _pd_yneg; //2
			uint16_t _pd_zpos; //2
			uint16_t _pd_zneg; //2 
			uint16_t _crc; //2 
			//Total = 26 bytes
		};
	};
	
	void computeCRC();

public:
	ADCSdata();
	void setStatus(uint8_t s);
	void setINAdata(INAdata data);
	void setSpeed(float s);
	void setIMUdata(IMUdata data);
	void setPDdata(PDdata_int data);
	uint8_t *getBytes();
	void clear();
	void send();
};

/* HARDWARE INIT FUNCTIONS ================================================== */

void initUSB(void);
void initUART(void);
void initI2C(void);

// fixed/float conversions
fixed5_3_t floatToFixed(float f);
float fixedToFloat(fixed5_3_t fix);

#endif