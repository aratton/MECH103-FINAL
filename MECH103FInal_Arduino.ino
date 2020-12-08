#include <BMP180MI.h>

//Alex Ratton
//Last Modified December 6 2020
//MECH103 Final Project
//This code reads data from the BMP180 sensor and writes the data to an SD card


//Librarys used
#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>
File myFile;
SFE_BMP180 pressure;
#define ALTITUDE 103.63 //Pleasanton, CA
void setup()
{
  Serial.begin(9600);
  Serial.println("REBOOT");

  // Initialize the sensor (it is important to get calibration values stored on the device).

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail");

    while (1); // Pause forever.
  }

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");

  }
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.

}
void loop() {
  myFile = SD.open("test.csv", FILE_WRITE);
  char status;
  double T, P, p0, a;
  //myFile.print("provided altitude: ");
  //myFile.print(ALTITUDE, 0);
  //myFile.print(" meters, ");
  //myFile.print(ALTITUDE * 3.28084, 0);
  //myFile.print(" feet");
  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    /*status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      myFile.print("temperature: ");
      myFile.print(T, 2);
      myFile.print(" deg C, ");
      myFile.print((9.0 / 5.0)*T + 32.0, 2);
      myFile.print(" deg F");
*/
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          // Print out the measurement:
          /*myFile.print("absolute pressure: ");
          myFile.print(P, 2);
          myFile.print(" mb, ");
          myFile.print(P * 0.0295333727, 2);
          myFile.print(" inHg");
*/
          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P, ALTITUDE); // 
/*          myFile.print("relative (sea-level) pressure: ");
          myFile.print(p0, 2);
          myFile.print(" mb, ");
          myFile.print(p0 * 0.0295333727, 2);
          myFile.print(" inHg");
*/
          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P, p0);
          myFile.print("computed altitude: ");
          myFile.print(a, 0);
          myFile.print(" meters, ");
          myFile.print(a * 3.28084, 0);
          myFile.print(" feet");

          Serial.println("computed altitude: ");
          Serial.print(a, 0);
          Serial.println(" meters, ");
          Serial.print(a * 3.28084, 0);
          Serial.println(" feet");
        }
        else myFile.print("error retrieving pressure measurement");
      }
      else myFile.print("error starting pressure measurement");
    }
    else myFile.print("error retrieving temperature measurement");
  
  //}else myFile.print("error starting temperature measurement");

  delay(500);  // Pause for 5 seconds.

  myFile.close();

}
