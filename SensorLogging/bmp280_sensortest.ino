/* This code is to use with Adafruit BMP280           (Metric)
 * It measures both temperature and pressure and it displays them on the Serial monitor with the altitude
 * It's a modified version of the Adafruit example code
 * Refer to www.surtrtech.com or SurtrTech Youtube channel
 */

#include <Adafruit_BMP280.h>
#include <SFE_BMP180.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
File myFile;

Adafruit_BMP280 bmp; // I2C Interface
float initPressure = 0.00;


void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 test"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

Serial.println("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");

  }
  Serial.println("initialization done.");

initPressure = (bmp.readPressure()/100.00);
Serial.print(F("Initial Pressure = "));
Serial.print((initPressure*100)*0.0002953);
Serial.println(" inHg");
}


void loop() {

    myFile = SD.open("test.csv", FILE_WRITE);
   
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
    
    delay(500);

    myFile.close();

}
