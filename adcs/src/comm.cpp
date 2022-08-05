#include "comm.h"

/* TEScommand METHODS ======================================================= */

/**
 * @brief      Constructs a new instance and sets empty as default state
 */
TEScommand::TEScommand()
{
	clear();
}

/**
 * @brief      Adds a byte to command
 *
 * @param[in]  b     Byte to be added to buffer, ignored if buffer is full
 */
void TEScommand::addByte(uint8_t b)
{
	_data[_bytes_received] = b;
	_bytes_received++;

	if (_bytes_received >= COMMAND_LEN)
	{
		_full = true;
		_bytes_received = 0;
	}
	else
	{
		_full = false;
	}
}

/**
 * @brief      Places bytes of input into the data field prior to transmission
 *
 * @param      bytes  Bytes to be added to the command for transmission
 */
void TEScommand::loadBytes(uint8_t *bytes)
{
	for (int i = 0; i < COMMAND_LEN; i++)
	{
		_data[i] = bytes[i];
	}
}

/**
 * @brief      Check if the command is full/ready to be sent/needs to be cleared before sending or receiving new command
 *
 * @return     True if full, False otherwise.
 */
bool TEScommand::isFull()
{
	return _full;
}

/**
 * @brief      The command stored in the structure.
 *
 * @return     The command.
 */
uint8_t TEScommand::getCommand()
{
	return _command;
}

/**
 * @brief      Check if the packet is still valid or has been compromised by a bit flipping
 *
 * @return     True if CRC is correct and packet is valid, False if packet should be discarded
 */
bool TEScommand::checkCRC()
{
	CRC16 crcGen;
	crcGen.add(_data, COMMAND_LEN-2);

	if (crcGen.getCRC() == _crc)
		return true;
	else
		return false;
}

/**
 * @brief      Empty the data field to receive/build a new command. When empty all fields are set to 0.
 */
void TEScommand::clear()
{
	for (int i = 0; i < COMMAND_LEN; i++)
		_data[i] = 0;

	_bytes_received = 0;
}

/* ADCSdata METHODS ========================================================= */

/**
 * @brief      Constructs a new instance and sets it to empty by default. Empty fields are 0 by default.
 */
ADCSdata::ADCSdata()
{
	clear();
}

/**
 * @brief      Sets the status, OK if all is good in our system, FUDGED if fake data or all 0's. See comm.h for possible values.
 *
 * @param[in]  s     System status as of this packet
 */
void ADCSdata::setStatus(uint8_t s)
{
	_status = s;
}

/**
 * @brief      Set the voltage and current values in data packet from an INAdata read.
 *
 * @param[in]  data  System power data to send to TES for monitoring.
 */
void ADCSdata::setINAdata(INAdata data)
{
	_voltage = floatToFixed(data.voltage);
	_current = (int8_t)data.current;
}

/**
 * @brief      Set the IMU data fields in the data packet to inform TES system of it's velocity and IMU functionality.
 *
 * @param[in]  data  Magnetometer and Gyroscope values.
 */
void ADCSdata::setIMUdata(IMUdata data)
{
	_magX = (int8_t)data.magX;
	_magY = (int8_t)data.magY;
	_magZ = (int8_t)data.magZ;

	_gyroX = floatToFixed(data.gyrX);
	_gyroY = floatToFixed(data.gyrY);
	_gyroZ = floatToFixed(data.gyrZ);
}

void ADCSdata::setPDdata(PDdata_int data)
{
	_pd_xpos = data.x_pos;
	_pd_xneg = data.x_neg;
	_pd_ypos = data.y_pos;
	_pd_yneg = data.y_neg;
	_pd_zpos = data.z_pos;
	_pd_zneg = data.z_neg;

}

/**
 * @brief      Compute CRC for validation of the packet
 */
void ADCSdata::computeCRC()
{
	CRC16 crcGen;
	crcGen.add(_data, PACKET_LEN-2);
	_crc = crcGen.getCRC();
}

/**
 * @brief      Get the data field
 *
 * @return     Pointer to the data field
 */
uint8_t* ADCSdata::getBytes()
{
	return _data;
}

/**
 * @brief      Clears the object by filling all data fields with 0.
 */
void ADCSdata::clear()
{
	for (int i = 0; i < PACKET_LEN; i++)
		_data[i] = 0;
}

/**
 * @brief      Send packet over UART connection
 */
void ADCSdata::send()
{
	computeCRC();
	SERCOM_UART.write((char*)_data, PACKET_LEN);
}

/* HARDWARE INIT FUNCTIONS ================================================== */

/**
 * @brief      Setup the USB serial connection at 115200 baud
 */
void initUSB(void)
{
	/**
     * Initialize USB connection to computer. Used to print debug messages.
     * Baud rate: 115200
     * Data bits: 8
     * Parity: none
     */
    SERCOM_USB.begin(115200);
    while (!SERCOM_USB);  // wait for initialization to complete
    SERCOM_USB.print("[system init]\tUSB interface initialized\r\n");
}

/**
 * @brief      Initialize UART connection to TES system at 115200 baud
 */
void initUART(void)
{
	/**
     * Initialize UART connection to satellite
     * Baud rate: 115200
     * Data bits: 8
     * Parity: odd (1 bit)
     */
    SERCOM_UART.begin(115200, SERIAL_8O1);
    while (!SERCOM_UART);  // wait for initialization to complete
	SERCOM_UART.setTimeout(10);
	#if DEBUG
	    SERCOM_USB.print("[system init]\tUART interface initialized\r\n");
	#endif
}

/**
 * @brief      Initializes I2C bus and set clock to 400000
 */
void initI2C(void)
{
	/**
     * Initialize I2C network
     * Clock: 400 kHz
     */
    SERCOM_I2C.begin();
    SERCOM_I2C.setClock(400000);
	#if DEBUG
		SERCOM_USB.print("[system init]\tI2C interface initialized\r\n");
	#endif
}

/* HELPER FUNCTIONS ========================================================= */

/**
 * @brief
 * Converts a floating-point number to a fixed-point number with 5 bits for the
 * integer part and 3 bits for the fraction part. Resulting data cannot be
 * properly interpreted until converted back into a float.
 * 
 * @param[in] f  Float to convert
 * 
 * @return Fixed-point conversion
 */
fixed5_3_t floatToFixed(float f)
{
    fixed5_3_t fix;
    fix = (uint8_t)(f * (1 << 3));
    return fix;
}

/**
 * @brief
 * Converts a fixed-point number with 5 integer bits and 3 fraction bits to a
 * floating-point number. Resulting data can be properly interpreted with no
 * extra conversions.
 * 
 * @param[in] fix  Fixed-point number to convert
 * 
 * @return Float conversion
 */
float fixedToFloat(fixed5_3_t fix)
{
    float f;
    f = ((float)fix) / (1 << 3);
    return f;
}