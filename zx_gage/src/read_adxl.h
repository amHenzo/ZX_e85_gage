#ifndef READ_ADXL_H
#define READ_ADXL_H
#include <SPI.h>
#include <Arduino.h>


/* ------------ adxl345 ----------------------*/
//Assign the Chip Select signal to pin 16
extern int CS_adxl;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
extern char POWER_CTL ;  //Power Control Register
extern char DATA_FORMAT ;
extern char DATAX0 ; //X-Axis Data 0
extern char DATAX1; //X-Axis Data 1
extern char DATAY0 ; //Y-Axis Data 0
extern char DATAY1 ; //Y-Axis Data 1
extern char DATAZ0 ; //Z-Axis Data 0
extern char DATAZ1 ; //Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
extern char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
extern int16_t x,y,z;
extern float roll;
extern float pitch;
extern float rollF,pitchF;

/* ------------ adxl345 ----------------------*/

void read_adxl();
int getSpriteBy(float pitch,float roll );
void readRegister(char registerAddress, int numBytes, char * values);
void writeRegister(char registerAddress, char value);
#endif
