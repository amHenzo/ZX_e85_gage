#include <Arduino.h>
#include <U8g2lib.h>
#include <DS3231.h>
#include "DHT.h"
#include "Car_texture.h"
#include "read_adxl.h"

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// DS3231
RTClib timer;
DateTime date;
char str_time[9];
char str_date[11];   
#define I2C_SDA 4  // GPIO0
#define I2C_SCL 21  // GPIO2

int CS_u8g2=5;
U8G2_ST7920_128X64_F_HW_SPI u8g2(U8G2_R0, /* CS=*/ CS_u8g2, /* reset=*/ 22); // lcd

#define DHTPIN 17
#define DHTTYPE DHT11
//DHTTYPE = DHT11, mais il existe aussi le DHT22 et 21 temp sensor

DHT dht(DHTPIN, DHTTYPE); 
#define ONBOARD_LED 2

unsigned long lastTime = 0;   // For timing FPS calculations
int frames = 0;               // Frame counter
float fps = 0;              

float humidity = 0;// dht.readHumidity(); 
float temp = 0; // dht.readTemperature();
int sec =0;

void readTempHumidity()
{
    humidity = dht.readHumidity(); 
    temp = dht.readTemperature();
}

void read_time()
{
  date = timer.now();
  sprintf(str_time, "%02d:%02d:%02d", date.hour(), date.minute(),date.second());
  sprintf(str_date, "%02d/%02d/%04d", date.day(), date.month(), date.year());
}

void printToDisplay()
{
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_profont10_mf);
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


  u8g2.setCursor(75, 7); u8g2.print(str_date);
  u8g2.setFont(u8g2_font_profont15_mf);
  u8g2.setCursor(2, 50);
  u8g2.print(str_time);
  
  sec = getSpriteBy(pitchF,rollF);

  u8g2.drawXBM(70, 10, 20, 64, bitmap_allArray[sec]);
  
  // Send buffer content to the display
  u8g2.sendBuffer();

  // Count the frame
  frames++;

  // Calculate FPS every second actions that is made one time every second too 
  if (millis() - lastTime >= 1000) {
    readTempHumidity();
    read_time();
    fps = frames;  // Frames in the last second
    frames = 0;    // Reset the frame counter
    lastTime = millis();  // Reset the timer
  }
}





void setup() {
  dht.begin();
  // lcd 800000
  u8g2.setBusClock(800000);//1000000 
  u8g2.begin();
  u8g2.setFont(u8g2_font_profont10_mf);// u8g2_font_ncenB08_tr // u8g2_font_boutique_bitmap_7x7_t_all  u8g2_font_5x8_mf u8g2_font_5x7_mf u8g2_font_04b_03_tr
  lastTime = millis(); 
  
  pinMode(ONBOARD_LED,OUTPUT);
  pinMode(CS_u8g2,OUTPUT);

  //digitalWrite(CS_u8g2, LOW);
  
  // adxl
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(115200);
  
  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS_adxl, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS_adxl, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  
  Wire.begin(I2C_SDA, I2C_SCL);
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
