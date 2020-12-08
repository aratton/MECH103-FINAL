/* This code is to use with Adafruit BMP280           (Metric)
 * It measures both temperature and pressure and it displays them on the Serial monitor with the altitude
 * It's a modified version of the Adafruit example code
 * Refer to www.surtrtech.com or SurtrTech Youtube channel
 */

#include <Adafruit_BMP280.h>
#include <SFE_BMP180.h>
#include <Adafruit_BMP280.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <SPI.h>
#include <SD.h>
File myFile;

Adafruit_BMP280 bmp; // I2C Interface
Adafruit_MPU6050 mpu;

//Initialize some global variables for altitude calculation
float initPressure = 0.00;

void setup() {
  Serial.begin(9600);
  
  Serial.println(F("BMP280 test"));

  //BMP
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


//MPU
 Serial.println("Adafruit MPU6050 test");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);


//SD CARD
Serial.println("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");

  }

//Redefine variables for altitude calculation, sets starting pressure as 0 ft altitude
initPressure = (bmp.readPressure()/100.00);
Serial.print(F("Initial Pressure = "));
Serial.print((initPressure*100)*0.0002953);
Serial.println(" inHg");

}


void loop() {

    myFile = SD.open("test.csv", FILE_WRITE);

//BMP
  /*
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure()/100); //displaying the Pressure in hPa, you can change the unit
    Serial.println(" hPa");
*/
    //Serial.println(F("Approx altitude"));
    Serial.print(3.28084*((bmp.readAltitude(initPressure)))); 
    Serial.println("'");

    myFile.print(3.28084*((bmp.readAltitude(initPressure))));
    myFile.print(",");

//MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  */

  //Acceleration Vector Through the Board
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print(" Roll Rate: ");
  Serial.print(g.gyro.x);
  Serial.print(", Pitch Rate: ");
  Serial.print(g.gyro.y);
  Serial.print(", Yaw Rate: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

    Serial.println("");

    delay(500);

    myFile.close();

}
