/****************************************************************
 * Library for interfacing with the DRV 10970 motor driver.
 *
 * @author: Garrett Wells
 * @date: 01/18/22
 *
 * Provides a configurable interface for working with the DRV 10970 motor driver. Initially written for University of Idaho senior capstone 2022 for use by the
 * Attitude adjustment team working for NASA.
 ****************************************************************/
#ifndef DRV_10970_H
#define DRV_10970_H

#include <Arduino.h>

// default pinout for the SAMD51
const int   MEN = A1,       // motor power enable pin
            DRV_FG = 6,     // frequency/rpm indication pin
            DRV_FR = 9,     // motor direction pin
            DRV_BRKMOD = 0, // brake mode (coast/brake), not currently available
            DRV_PWM = 10,   // pwm output pin
            DRV_RD = 5;     // lock indication pin

enum MotorDirection {   
                    CW=0,  // clockwise
                    CCW=1,  // counter clockwise
                    IDLE=2  // don't move
                    };

class DRV10970 {
    private:
        int MEN, FG, FR, BRKMOD, PWM, RD; // interface pins
    public:
        DRV10970(void){}
        DRV10970(int men, int fg, int fr, int brkmod, int pwm, int rd);
		void init(void);
        void run(MotorDirection dir, int dc); // drive motor in direction at dutycycle dc
        void stop(); // stop motor driver and put in low power state
        int readRPS(bool); // returns the rpm of motor spindle
        bool spindleFree(); // returns true if motor spindle is free to spin
 };
#endif
