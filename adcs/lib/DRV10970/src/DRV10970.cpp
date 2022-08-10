/****************************************************************
 * Library for interfacing with the DRV 10970 motor driver.
 *
 * @author: Garrett Wells
 * @date: 01/18/22
 *
 * Provides a configurable interface for working with the DRV 10970 motor driver. Initially written for University of Idaho senior capstone 2022 for use by the
 * Attitude adjustment team working for NASA.
 ****************************************************************/
#include "DRV10970.h"

/**
 * @brief      Constructs a new instance.
 *
 * @param[in]  men     The motor enable pin
 * @param[in]  fg      The frequency indication pin
 * @param[in]  fr      Motor direction pin
 * @param[in]  brkmod  Brake mode pin
 * @param[in]  pwm     The pwm signal pin
 * @param[in]  rd      Lock indication pin
 */
DRV10970::DRV10970(int men, int fg, int fr, int brkmod, int pwm, int rd){

    MEN = men;          // motor enable pin
    FG = fg;            // frequency indication pin
    FR = fr;            // motor direction control
    BRKMOD = brkmod;    // brake mode setting pin
    PWM = pwm;          // variable duty cycle pwm input pin for speed control
    RD = rd;            // lock indication pin
}

/**
 * @brief      Initializes pin modes and states
 */
void DRV10970::init(void)
{
	// set pin modes
    pinMode(MEN, OUTPUT);
    digitalWrite(MEN, LOW); // turn off power to the motor initially

    pinMode(FG, INPUT);

    pinMode(FR, OUTPUT);
    digitalWrite(FR, LOW);

    //pinMode(BRKMOD, OUTPUT); // this pin not currently exposed
    pinMode(PWM, OUTPUT);
    stop(); // make sure the motor output pin is not floating initially

    pinMode(RD, INPUT);
}

/**
 * @brief      Drive the pwm pin at a specified duty cycle
 *
 * @param[in]  dir   The direction the spindle should rotate
 * @param[in]  dc    The duty cycle to run the motor at
 */
void DRV10970::run(MotorDirection dir, int dc){
    // enable power to the motor
    digitalWrite(MEN, HIGH);
    // write direction
    digitalWrite(FR, dir);
    // write PWM
    analogWrite(PWM, dc);
    #ifdef TEST_INDEPENDENT
        SERCOM_USB.println("I'm INDEPENDENT!!");
    #endif
}

/**
 * @brief      Set the pwm pin output to low and disable power to the motor using motor enable
 */
void DRV10970::stop(){

    digitalWrite(MEN, LOW); // disable power to the motor
    analogWrite(PWM, LOW);  // pull pwm low
    
}
/**
 * @brief      Read the number of ticks on the motor frequency pin in 50 milliseconds
 *
 * @param[in]  debug  print debug statements
 *
 * @return     Number of ticks observed in 50 milliseconds -> careful, we are approaching the nyquist limit...
 */
 int DRV10970::readRPS(bool debug=false){    
    //note, this function should return a double, to facilitate resolution; ok for the time being
    long int t0 = millis(); // read start time
    long int cT = millis(); // current time
    int gap = 100;                    // milliseconds to measure for
    int toggles = 0;                 // set number of electrical toggles
    int prev_state = digitalRead(FG);     // state variable
    int cState;
    double frequency;
    while(cT - t0 < gap){
        cState = digitalRead(FG); // the current state of the FG pin

        if(debug) Serial.println(cState);

        if(prev_state != cState){ // if has toggled, count it
            prev_state = cState;
            toggles++;
        }
        cT = millis();
    }

    frequency = (toggles/6.0) / 0.100;    //frequency in Hz
    frequency = frequency * 60;

    //We are assuming here that 1 "toggle" is equal to 1 full rotation of the spindle. If we record for 1s, you can multiply the
    //value we get by 60; which would give us the revolutions per min (RPM). *IN THEORY*
    //toggles = toggles*60;


    return frequency;  
}

/**
 * @brief      Check if the motor spindle is rotating (locked)
 *
 * @return     True if not locked, false otherwise
 */
bool DRV10970::spindleFree(){
    return digitalRead(RD) == HIGH;
}
