/**
 * @defgroup   GLOBAL_DEFINITIONS global_definitions.h
 *
 * @brief      This file implements global definitions.
 *
 * @author     Garrett Wells, Parker Piedmont
 * @date       2022
 */
#ifndef GLOBAL_DEFINITIONS_H
#define GLOBAL_DEFINITIONS_H

// set to 1 for the ADCS to print to the usb serial connection from SAMD51
#define DEBUG 1

// create more descriptive names for serial interfaces
#define SERCOM_USB Serial
#define SERCOM_UART Serial1
#define SERCOM_I2C Wire
#define AD0_VAL 1

//Actuator Pin Definitions 
#define MTX1_F_PIN 24
#define MTX1_R_PIN 23
#define MTX2_F_PIN 22
#define MTX2_R_PIN 4
#define MEN_PIN A1
#define BEN_PIN A5
#define PWM_PIN 10 
#define FG_PIN 6
#define FR_PIN 9
#define RD_PIN 5



#endif
