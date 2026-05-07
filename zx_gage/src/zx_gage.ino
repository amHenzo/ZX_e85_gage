#include <Arduino.h>
#include <U8g2lib.h>
#include "DHT.h"
#include "Car_texture.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

int CS_u8g2=5;
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* CS=*/ CS_u8g2, /* reset=*/ 22); // lcd

#define DHTPIN 17
#define DHTTYPE DHT11
//DHTTYPE = DHT11, mais il existe aussi le DHT22 et 21 temp sensor

DHT dht(DHTPIN, DHTTYPE); 
#define ONBOARD_LED  2

/* ------------ RTC (DS3231) ----------------------*/
#define DS3231_ADDR 0x68
#define DS3231_SEC_REG 0x00

uint8_t rtc_hour = 0;
uint8_t rtc_minute = 0;
uint8_t rtc_second = 0;
uint8_t rtc_day = 1;
uint8_t rtc_month = 1;
uint16_t rtc_year = 2024;
unsigned long last_rtc_read = 0;

unsigned long lastTime = 0;   // For timing FPS calculations

uint8_t bcd_to_decimal(uint8_t bcd) {
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

void initRTC() {
    Wire.begin(21, 4); // SDA=GPIO21, SCL=GPIO4
    Wire.setClock(100000); // 100kHz I2C speed
    Serial.println("[RTC] DS3231 initialized on I2C");
}

void readRTC() {
    // Limit reads to 1 per 2 seconds to avoid blocking
    if (millis() - last_rtc_read < 2000) {
        return;
    }
    last_rtc_read = millis();

    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(DS3231_SEC_REG);
    Wire.endTransmission(false);  // Do not release bus

    // Request with short timeout
    uint8_t available = Wire.requestFrom((uint8_t)DS3231_ADDR, (uint8_t)7, (uint8_t)1);

    if (available >= 7) {
        uint8_t sec_raw = Wire.read();
        uint8_t min_raw = Wire.read();
        uint8_t hour_raw = Wire.read();
        Wire.read(); // DAY of week - ignore
        uint8_t date_raw = Wire.read();
        uint8_t month_raw = Wire.read();
        uint8_t year_raw = Wire.read();

        rtc_second = bcd_to_decimal(sec_raw & 0x7F);
        rtc_minute = bcd_to_decimal(min_raw & 0x7F);
        rtc_hour = bcd_to_decimal(hour_raw & 0x3F);
        rtc_day = bcd_to_decimal(date_raw & 0x3F);
        rtc_month = bcd_to_decimal(month_raw & 0x1F);
        rtc_year = 2000 + bcd_to_decimal(year_raw);
    }
}
/* ------------ RTC ----------------------*/
int frames = 0;               // Frame counter
float fps = 0;                // Frames per second

float humidity = 0;// dht.readHumidity(); 
float temp = 0; // dht.readTemperature();
int sec =0;

/* ------------ adxl345 ----------------------*/
//Assign the Chip Select signal to pin 16
int CS_adxl=16;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;  //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int16_t x,y,z;
float roll = 0;
float pitch = 0;
float rollF,pitchF=0;

/* ------------ adxl345 ----------------------*/


void printToDisplay()
{
  u8g2.clearBuffer();
  u8g2.setCursor(0, 7);
  u8g2.print("Frame Test");
  u8g2.setCursor(55, 7);
  u8g2.print(frames); 
  // Draw FPS value
  u8g2.setCursor(0, 15);
  u8g2.print(fps, 1);
  u8g2.setCursor(19, 15);
  u8g2.print(" FPS");
  // bumidity
  u8g2.setCursor(0, 23);
  u8g2.print(humidity, 1);
  u8g2.setCursor(25, 23);
  u8g2.print("\xF7""H"); // for raw value of %
  u8g2.setCursor(0, 31);
  u8g2.print(temp, 1);
  u8g2.setCursor(25, 31);
  u8g2.print("\xB0""C"); // for raw value of °


  sec = getSpriteBy(pitchF,rollF);

  u8g2.drawXBM(70, 10, 20, 64, bitmap_allArray[sec]);
  
  // Display RTC date at top right
  u8g2.setCursor(75, 7);
  if(rtc_day < 10) u8g2.print("0");
  u8g2.print(rtc_day);
  u8g2.print("/");
  if(rtc_month < 10) u8g2.print("0");
  u8g2.print(rtc_month);
  u8g2.print("/");
  u8g2.print(rtc_year);
  
  // Display RTC time at bottom right
  u8g2.setCursor(80, 63);
  if(rtc_hour < 10) u8g2.print("0");
  u8g2.print(rtc_hour);
  u8g2.print(":");
  if(rtc_minute < 10) u8g2.print("0");
  u8g2.print(rtc_minute);
  
  // Send buffer content to the display
  u8g2.sendBuffer();

  // Count the frame
  frames++;

  // Calculate FPS every second
  if (millis() - lastTime >= 1000) {
    readTempHumidity();
    readRTC();  // Read RTC once per second
    fps = frames;  // Frames in the last second
    frames = 0;    // Reset the frame counter
    lastTime = millis();  // Reset the timer
  }
}

void readTempHumidity()
{
    humidity = dht.readHumidity(); 
    temp = dht.readTemperature();
}

void read_adxl()
{
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  x = ((int16_t)values[1] << 8) | (int16_t)values[0];  // Valeur d'accélération X
  y = ((int16_t)values[3] << 8) | (int16_t)values[2];  // Valeur d'accélération Y
  z = ((int16_t)values[5] << 8) | (int16_t)values[4];  // Valeur d'accélération Z

  roll = atan(y / sqrt(pow(x, 2) + pow(z, 2))) * 180 / PI;
  pitch = atan(-1 * x / sqrt(pow(y, 2) + pow(z, 2))) * 180 / PI;

  // filter low
  rollF = 0.94 * rollF + 0.06 * roll;
  pitchF = 0.94 * pitchF + 0.06 * pitch;

  //Print the results to the terminal.
  Serial.print(x, DEC);
  Serial.print(',');
  Serial.print(y, DEC);
  Serial.print(',');
  Serial.print(z, DEC);
  Serial.print(',');
  Serial.print(roll,2);
  Serial.print(',');
  Serial.println(pitch,2);

}
//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS_adxl, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS_adxl, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS_adxl, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS_adxl, HIGH);
}

int getSpriteBy(float pitch,float roll )
{
  if( abs(pitch) < 15 && abs(roll) < 15)
     return 2;
   else
   {
      if ( abs(pitch) > abs(roll))
      {
        if(pitch < 0 )
         return 1;
        else
          return 3;
      }
      else
      {
        if(roll < 0 )
          return 0;
        else
          return 4;
      }
   }
   return 2;
}

void setup(void) {
  dht.begin();
  // lcd 800000
  u8g2.setBusClock(800000);//1000000 
  u8g2.begin();
  u8g2.setFont(u8g2_font_profont10_mf);// u8g2_font_ncenB08_tr // u8g2_font_boutique_bitmap_7x7_t_all  u8g2_font_5x8_mf u8g2_font_5x7_mf u8g2_font_04b_03_tr
  lastTime = millis(); 
  
  pinMode(ONBOARD_LED,OUTPUT);
  pinMode(CS_u8g2,OUTPUT);

  //digitalWrite(CS_u8g2, LOW);
  
  // RTC initialization
  initRTC();
  readRTC();
  
  // adxl
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS_adxl, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS_adxl, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  
}

void loop() {
  
  digitalWrite(ONBOARD_LED, HIGH);
  //digitalWrite(CS_u8g2, HIGH);
  
  printToDisplay();
  //digitalWrite(CS_u8g2, LOW);
  
  digitalWrite(CS_adxl, HIGH);
  read_adxl();
  digitalWrite(CS_adxl, LOW);
  digitalWrite(ONBOARD_LED, LOW);

}
