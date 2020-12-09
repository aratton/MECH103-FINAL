

//BAROMETER///////////////
#include <Adafruit_BMP280.h>
#include <SFE_BMP180.h>
#include <Adafruit_BMP280.h>

//ORIENTATION/////////////
/*#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
*/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

//SD CARD///////////
#include <SPI.h>
#include <SD.h>
File myFile;

Adafruit_BMP280 bmp; // I2C Interface
//Adafruit_MPU6050 mpu;
MPU6050 mpu;

//Initialize some global variables for altitude calculation
float initPressure = 0.00;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}






void setup() {
  Serial.begin(9600);
  
  Serial.println(F("BMP280 test"));

  //BMP//////////////
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

/*
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
*/


//MPU/////////////////////

// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
 
mpu.initialize();
Serial.begin(9600);
devStatus = mpu.dmpInitialize();
 
// supply your own gyro offsets here, scaled for min sensitivity
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
 
// make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
 
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
 
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
 
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
 
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

//SD CARD////////////

Serial.println("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
  }

//Redefine variables for altitude calculation, sets starting pressure as 0 ft altitude/////////
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


//MPU

  //sensors_event_t a, g, temp;
  //mpu.getEvent(&a, &g, &temp);
  /*
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  */

  //ACCELERATION Vector Through the Board
  /*
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");
  */

/*
  //ROLL RATES
  //Serial.print("Roll Rate: ");
  Serial.print(g.gyro.x);
  Serial.print(",");
  //Serial.print(", Pitch Rate: ");
  Serial.print(g.gyro.y);
  //Serial.print(", Yaw Rate: ");
  Serial.print(",");
  Serial.print(g.gyro.z);
  //Serial.println(" rad/s");
    Serial.println("");

    myFile.print(g.gyro.x);
    myFile.print(",");
    myFile.print(g.gyro.y);
    myFile.print(",");
    myFile.println(g.gyro.z);
    //myFile.print(" "); //SHOULD RETURN A NEW ROW
*/

//NEW GYRO CODE////////////
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
 
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);
 
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
 
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
 
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));
 
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //ALTITUDE//////////
    Serial.print("Altitude: ");
    Serial.print(3.28084*((bmp.readAltitude(initPressure)))); 
    Serial.println("'");

    myFile.print(3.28084*((bmp.readAltitude(initPressure))));
    myFile.print(",");
    
    //Serial.print("Yaw: ");
   // Serial.println(ypr[0] * 180/M_PI);
    Serial.print("Pitch: ");
    Serial.println(ypr[1] * 180/M_PI);
    Serial.print("Roll: ");
    Serial.println(ypr[2] * 180/M_PI);

    myFile.print(ypr[1] * 180/M_PI);
    myFile.print(",");
    myFile.println(ypr[2] * 180/M_PI);
    
    delay(100);
  }
  myFile.close();
}
