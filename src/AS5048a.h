#ifndef AS5048A_LIB_H
#define AS5048A_LIB_H

#include <SPI.h>
#include "FOCutils.h"
#include "Sensor.h"

#define NUM_BITS 14
#define ANGLE_REG 0x3FFF
#define MAG_REG 0x3FFE
#define DIAG_AGC_REG 0x3FFD

struct SensorStatus
{
  /* data */
  bool ocf;
  bool cof;
  bool comp_lo;
  bool comp_hi;
  float agc_val;
};

class AS5048a: public Sensor{
 public:
    /**
     *  AS5048a class constructor
     * @param cs  SPI chip select pin 
     */
    AS5048a(int cs);

    /** sensor initialise pins */
    void init() override;

    // implementation of abstract functions of the Sensor class
    /** get current angle (rad) */
    float getAngle() override;
    /** get current angular velocity (rad/s) **/
    float getVelocity() override;
    /**
     *  set current angle as zero angle 
     * return the angle [rad] difference
     */
    float initRelativeZero() override;
    /**
     *  set absolute zero angle as zero angle
     * return the angle [rad] difference
     */
    float initAbsoluteZero() override;
    /** returns 1 because it is the absolute sensor */
    int hasAbsoluteZero() override;
    /** returns 0  maning it doesn't need search for absolute zero */
    int needsAbsoluteZeroSearch() override;
    
    // additional function used by AS5048a
    /** get Magnitude information of the magnetic field **/
    float getMagnitude();
    /** get Magnitude information of the magnetic field **/
    SensorStatus getStatus();

  private:
    float cpr; //!< Maximum range of the magnetic sensor
    // spi variables
    int angle_register; //!< SPI angle register to read
    int chip_select_pin; //!< SPI chip select pin
	  SPISettings settings; //!< SPI settings variable
    // spi functions
    /** Stop SPI communication */
    void close(); 
    /** Read one SPI register value */
    word read(word angle_register);
    /** Calculate parity value  */
    byte spiCalcEvenParity(word value);


    word zero_offset; //!< user defined zero offset
    /**
     * Function getting current angle register value
     * it uses angle_register variable
     */
    int getRawCount();

    // total angle tracking variables
    float full_rotation_offset; //!<number of full rotations made
    float angle_data_prev; //!< angle in previous position calculation step

    // velocity calculation variables
    float angle_prev; //!< angle in previous velocity calculation step
    long velocity_calc_timestamp; //!< last velocity calculation timestamp

};


#endif
