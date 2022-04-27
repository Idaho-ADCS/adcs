#include "flags.h"
#include "rtos/rtos_helpers.h"
#include "rtos/rtos_tasks.h"
#include "comm.h"
#include "supportFunctions.h"

#include "FreeRTOS_SAMD51.h"

extern QueueHandle_t modeQ;
extern DRV10970 flywhl;

void state_machine_transition(uint8_t mode)
{
	uint8_t curr_mode = CMD_STANDBY; // standby by default
	// get the current state to compare against
	xQueuePeek(modeQ, &curr_mode, 0);
	// make sure we are entering a new state

#if DEBUG
	char debug_str[16];
#endif

	if (mode == curr_mode) // if not, exit
	{
		return;
	}

	flywhl.stop();

	bool command_is_valid = true;

	switch (mode)
	{
	case CMD_HEARTBEAT:
		// do test command stuff
#if DEBUG
		SERCOM_USB.print("[mode switch]\tEntering HEARTBEAT mode\r\n");
#endif
		break;

	// WARNING: if you switch out of a test mode the test will still be on the stack
	// 	on the plus side, the task won't run though
	case CMD_TST_BASIC_MOTION:
	case CMD_TST_BASIC_AD:
	case CMD_TST_BASIC_AC:
	case CMD_TST_SIMPLE_DETUMBLE:
	case CMD_TST_SIMPLE_ORIENT:
	case CMD_TST_PHOTODIODES:
#if DEBUG
		SERCOM_USB.print("[mode switch]\tEntering TEST mode\r\n");
#endif

// #if RTOS_TEST_SUITE
		// create_test_tasks();
// #endif
		break;

	case CMD_STANDBY:
#if DEBUG
		SERCOM_USB.print("[mode switch]\tEntering STANDBY mode\r\n");
#endif
		// print heartbeat regularly turn off actuators
		// if(magnetorquer_on){
		// 	// TODO: SM turn magnetorquer off
		// }
		// if(flywhl_is_spinning){
		// 	// TODO: SM turn off the flywheel
		// }
		break;

	// TODO: SM fill out the other modes with functional code
	case CMD_ORIENT_DEFAULT: // should be orienting to something like X+
	case CMD_ORIENT_X_POS:
	case CMD_ORIENT_Y_POS:
	case CMD_ORIENT_X_NEG:
	case CMD_ORIENT_Y_NEG:
#if DEBUG
		SERCOM_USB.print("[mode switch]\tEntering ORIENT mode\r\n");
#endif
		break;

	default: // do nothing
		command_is_valid = false;
#if DEBUG
		// convert int to string for USB monitoring
		sprintf(debug_str, "0x%02x", mode);
		SERCOM_USB.print("[mode switch]\tUnknown command: ");
		SERCOM_USB.print(debug_str);
		SERCOM_USB.print("\r\n");
#endif
	}

	if (command_is_valid)
	{
		xQueueOverwrite(modeQ, (void *)&mode); // enter specified mode

		// TODO: SM init the new mode, maybe turn off an unneeded actuator? clear some data?
	}
}

void create_test_tasks(void)
{
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tInitializing RTOS test suite\r\n");
#endif

	xTaskCreate(photodiode_test, "PHOTODIODE TEST", 256, NULL, 1, NULL);
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tCreated photodiode test task\r\n");
#endif

	xTaskCreate(basic_motion, "BASIC MOTION", 256, NULL, 1, NULL);
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tCreated basic motion task\r\n");
#endif

// 	xTaskCreate(basic_attitude_determination, "BASIC AD", 256, NULL, 1, NULL);
// #if DEBUG
// 	SERCOM_USB.print("[rtos]\t\tCreated basic attitude determination task\r\n");
// #endif

// 	xTaskCreate(basic_attitude_control, "BASIC AC", 256, NULL, 1, NULL);
// #if DEBUG
// 	SERCOM_USB.print("[rtos]\t\tCreated basic attitude control task\r\n");
// #endif

	xTaskCreate(simple_detumble, "SIMPLE DETUMBLE", 256, NULL, 1, NULL);
#if DEBUG
	SERCOM_USB.print("[rtos]\t\tCreated simple detumble task\r\n");
#endif

// 	xTaskCreate(simple_orient, "SIMPLE ORIENT", 256, NULL, 1, NULL);
// #if DEBUG
// 	SERCOM_USB.print("[rtos]\t\tCreated simple orient task\r\n");
// #endif

#if DEBUG
	SERCOM_USB.print("[rtos]\t\tInitialized RTOS test suite\r\n");
#endif
}

// get direction the adcs should turn to align X+ with the light source
//
// @param[in]  vals  The sensor values, unscaled
//
// @return     The direction to turn the adcs (clockwise or counter-clockwise)
//
MotorDirection getDirection(PDdata vals){
	uint8_t max=0; // channel receiving the most light
	float max_val=-1;
	// calculate side/channel receiving most light, ignore +/-Z so only look at first 4 values
	for(int i=0; i < 4; i++){
		if(vals.data[i] >= max_val){
			// save channel and value
			max_val = vals.data[i];
			max = i;
		}
	}

	#if DEBUG 
		SERCOM_USB.print("getDirection found max on channel = ");
		SERCOM_USB.println(max);
	#endif

	// translate to coordinates and calculate direction to spin
	switch(max){
		case X_POS:
			return IDLE;
		case X_NEG:
			return FWD;
		case Y_POS:
			return REV;
		case Y_NEG:
			return FWD;
		case Z_POS:
		case Z_NEG:
			return IDLE;
	}
} 