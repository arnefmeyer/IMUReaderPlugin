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
enum status_t {WAITING=0, RUNNING=1, RECORDING=2};
int status;

// read magnetometer data? set to false if not needed 
// to speed up things
bool readMagnetometer = true;

// precision used for converting float to string values
int precision = 6;


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
}


void loop()
{
  if (status > 0)
  {
    lsm.readAccel();
    lsm.readGyro();
	if (readMagnetometer)
	{
  		lsm.readMag();
	}

    if (status > 1)
    {
      digitalWrite(triggerPin, HIGH);
    }

    if (readMagnetometer)
    {
      Serial.println(String(status) + " " 
                     + String(millis()) + " " 
                     + String(lsm.accelData.x, precision) + " " 
                     + String(lsm.accelData.y, precision) + " " 
                     + String(lsm.accelData.z, precision) + " " 
                     + String(lsm.gyroData.x, precision) + " " 
                     + String(lsm.gyroData.y, precision) + " " 
                     + String(lsm.gyroData.z, precision) + " "
                     + String(lsm.magData.x, precision) + " "
                     + String(lsm.magData.y, precision) + " "
                     + String(lsm.magData.z, precision));
    }
    else
    {
      // fill magnetometer values with zeros
      Serial.println(String(status) + " " 
                   + String(millis()) + " " 
                   + String(lsm.accelData.x, precision) + " " 
                   + String(lsm.accelData.y, precision) + " " 
                   + String(lsm.accelData.z, precision) + " " 
                   + String(lsm.gyroData.x, precision) + " " 
                   + String(lsm.gyroData.y, precision) + " " 
                   + String(lsm.gyroData.z, precision) + " "
                   + String(0., precision) + " "
                   + String(0., precision) + " "
                   + String(0., precision));
    }

    if (status > 1)
    {
      delay(1);
      digitalWrite(triggerPin, LOW);
    }

    delay(1);
  }
  else
  {
    digitalWrite(triggerPin, LOW);
    delay(10);
  }
}


void serialEvent() {
  while (Serial.available()) {
    status = (int)Serial.parseInt();
  }
}


