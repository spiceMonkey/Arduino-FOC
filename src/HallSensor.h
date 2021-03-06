#ifndef HALL_SENSOR_LIB_H
#define HALL_SENSOR_LIB_H

#include "Arduino.h"
#include "FOCutils.h"
#include "Sensor.h"


class HallSensor: public Sensor{
 public:
    /**
    HallSensor class constructor
    @param encA  HallSensor B pin
    @param encB  HallSensor B pin
    @param encC  HallSensor B pin
    @param pp  pole pairs  (e.g hoverboard motor has 15pp and small gimbals often have 7pp)
    @param index index pin number (optional input)
    */
    HallSensor(int encA, int encB, int encC, int pp);

    /** HallSensor initialise pins */
    void init();
    /**
     *  function enabling hardware interrupts for the HallSensor channels with provided callback functions
     *  if callback is not provided then the interrupt is not enabled
     * 
     * @param doA pointer to the A channel interrupt handler function
     * @param doB pointer to the B channel interrupt handler function
     * @param doIndex pointer to the Index channel interrupt handler function
     * 
     */
    void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr, void(*doC)() = nullptr);
    
    //  HallSensor interrupt callback functions
    /** A channel callback function */
    void handleA();
    /** B channel callback function */
    void handleB();
    /** C channel callback function */
    void handleC();
    
    
    // pins A and B
    int pinA; //!< HallSensor hardware pin A
    int pinB; //!< HallSensor hardware pin B
    int pinC; //!< HallSensor hardware pin C

    // HallSensor configuration
    Pullup pullup; //!< Configuration parameter internal or external pullups
    float cpr;//!< HallSensor cpr number

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float getAngle();
    /**  get current angular velocity (rad/s) */
    float getVelocity();
    /** 
     *  set current angle as zero angle 
     * return the angle [rad] difference
     */
    float initRelativeZero();
    /**
     * set index angle as zero angle
     * return the angle [rad] difference
     */
    float initAbsoluteZero();
    /**
     *  returns 0 if it has no index 
     * 0 - HallSensor without index
     * 1 - HallSensor with index 
     */
    int hasAbsoluteZero();
    /**
     * returns 0 if it does need search for absolute zero
     * 0 - HallSensor without index 
     * 1 - ecoder with index
     */
    int needsAbsoluteZeroSearch();

    // whether last step was CW (+1) or CCW (-1) direction
    Direction direction;

    // the current 3bit state of the hall sensors
    int state;

    volatile float angle; // rad/s
    volatile float velocity; // rad/s

  private:
    
    Direction decodeDirection(int oldState, int newState);
    void updateState();

    volatile long pulse_counter;//!< current pulse counter
    volatile long pulse_timestamp;//!< last impulse timestamp in us
    volatile int A_active; //!< current active states of A channel
    volatile int B_active; //!< current active states of B channel
    volatile int C_active; //!< current active states of C channel

    // velocity calculation variables
    // float prev_Th, pulse_per_second;
    volatile long prev_pulse_counter, prev_timestamp_us;
    
};


#endif
