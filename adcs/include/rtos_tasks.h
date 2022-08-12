/**
 * @defgroup   RTOS_TASKS rtos_tasks.cpp
 *
 * @brief      This file implements rtos tasks for various modes of operation.
 *
 * @author     Garrett Wells, Parker Piedmont, MMR
 * @date       2022
 */
#ifndef __RTOS_TASKS_H__
#define __RTOS_TASKS_H__

#include <global_definitions.h>
#include <comm.h>
#include <actuators.h>
#include <FreeRTOS_SAMD51.h>

extern DRV10970 flywhl;
extern ZXMB5210 Mtx1;
extern ZXMB5210 Mtx2;
extern ICM_20948_I2C IMU1;
extern QueueHandle_t modeQ;
extern QueueHandle_t IMUq;
extern SemaphoreHandle_t IMUsemphr;

void state_machine_transition(uint8_t cmd);
void create_test_tasks(void);

MotorDirection getDirection(PDdata); // get direction the adcs should turn to align X+ with the light source

// RTOS TASKS /////////////////////////////////////////////////////
void receiveCommand(void *pvParameters);
void heartbeat(void *pvParameters);
void photodiode_test(void *pvParameters);

void basic_motion(void* pvParameters);
void basic_attitude_determination(void *pvParameters);
void basic_attitude_control(void *pvParameters);
void simple_detumble(void *pvParameters);
void simple_orient(void *pvParameters);

void basic_mtx(void *pvParameters); //added 07252022; MMR
void basic_bldc(void *pvParameters);    //added 07252022; MMR

#endif