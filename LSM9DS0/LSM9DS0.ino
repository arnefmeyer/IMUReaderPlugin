/*
 * 
 * Tested with a Teensy 3.2 but should also work for other Teensy/Arduino models
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

// i2c
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();

// CMOS/TTL trigger pin
int triggerPin = 13;

// processing status
enum status_t {WAITING=1, RUNNING=2, RECORDING=3};
int status;
unsigned long counter;

// precision used for converting float to string values
int precision = 12;


void setupSensor()
{
  // Accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);

  // Gyroscope range
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);

  // Magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  // lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);
}


void setup() 
{
  while(!Serial) ;
  Serial.begin(115200);

  pinMode(triggerPin, OUTPUT);

  if (!lsm.begin())
  {
    while (1);
  }  
  
  setupSensor();
  status = 0;
  counter = 0;
}


void loop()
{

  if (Serial.available())
  {
    int val = (int)Serial.parseInt();
    if (val > 0)
    {
      status = val;
    }
  }

  if (status > WAITING)
  {
    lsm.readAccel();
    lsm.readGyro();
  	lsm.readMag();

    if (status > 1)
    {
      digitalWrite(triggerPin, HIGH);
      counter++;
    }

    Serial.println(String(status) + "," 
                   + String(counter) + ","
                   + String(millis()) + "," 
                   + String(lsm.accelData.x, precision) + "," 
                   + String(lsm.accelData.y, precision) + "," 
                   + String(lsm.accelData.z, precision) + "," 
                   + String(lsm.gyroData.x, precision) + "," 
                   + String(lsm.gyroData.y, precision) + "," 
                   + String(lsm.gyroData.z, precision) + ","
                   + String(lsm.magData.x, precision) + ","
                   + String(lsm.magData.y, precision) + ","
                   + String(lsm.magData.z, precision));

    delay(1);

    if (status > 1)
    {
      digitalWrite(triggerPin, LOW);
    }
  }
  else
  {
    digitalWrite(triggerPin, LOW);
    delay(10);
  }
}

