/*
  Arduino LSM9DS1 - Simple Accelerometer
  This example reads the acceleration values from the LSM9DS1
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.
  The circuit:
  - Arduino Nano 33 BLE Sense
  created 10 Jul 2019
  by Riccardo Rizzo
  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>

float ax, ay, az;
float gx, gy, gz;
float I_gz;
float mx, my, mz;

float avg_gz =0;
long gz_time = 0;
float gz_deg = 0;   //degrees in the z direction
float gz_vel_old = 0;   //set initial gz value to 0 until it's set in the loop

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");
  
  float sum_gz = 0;

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.print("Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Magnetic Field in uT");
  Serial.println("X\tY\tZ");
  
  unsigned long StartTime = millis();

  for (int i = 0; i<238 ;i++)
  {
     if (IMU.gyroscopeAvailable()) 
     {
        IMU.readGyroscope(gx, gy, gz);
     }
     sum_gz = sum_gz + gz;
     delay(8.4);  //delay of 8 ms between readings
  }
  unsigned long EndTime = millis();
  avg_gz = sum_gz/238;
  
  unsigned long EllapsedTime = EndTime - StartTime;
  
  Serial.println("Start Time = ");
  Serial.println(StartTime);
  Serial.println();
  Serial.println("End Time = ");
  Serial.println(EndTime);
  Serial.println();
  Serial.println("Time Ellapsed = ");
  Serial.println(EllapsedTime);
  Serial.println();
  Serial.println("Gz sum = ");
  Serial.println(sum_gz);
  Serial.println();
  Serial.println("Gz average = ");
  Serial.println(avg_gz);
  Serial.println();
}

void loop() 
{
   unsigned long newTime = millis();
   unsigned long deltaTime = newTime - gz_time;
   float deltaTimeS = deltaTime/1000;
   float new_gz_vel;
   float avg_gz_vel;
  
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }

 // Serial.print(newTime);Serial.print("\t");
 // Serial.print(deltaTime);Serial.print("\t");
 // Serial.print(deltaTimeS);Serial.print("\t");

//finding the average angular velocity

  if (gz_time != 0)
  {
    avg_gz_vel = (gz + gz_vel_old)/2;  //averaging the current gz velocity minus
    gz_deg = gz_deg + ((avg_gz_vel * deltaTime))/1000; //integrating the average ang vel 
    Serial.print(gz_deg);
    Serial.println();
  }

  gz_time = newTime;
  gz_vel_old = gz;
 // Serial.print(gz_time);Serial.print("\t");
 // Serial.println(); 
  
  
//commented out for now, just interested in the gyroscope z degrees
/*
  Serial.print("Acc: ");
  Serial.print(ax);
  Serial.print('\t');
  Serial.print(ay);
  Serial.print('\t');
  Serial.print(az);
  Serial.print('\t');

  Serial.print("Gyr: ");
  Serial.print(gx);
  Serial.print('\t');
  Serial.print(gy);
  Serial.print('\t');
  Serial.print(gz);
  Serial.print('\t');

  Serial.print("Mag: ");
  Serial.print(mx);
  Serial.print('\t');
  Serial.print(my);
  Serial.print('\t');
  Serial.println(mz); 
*/

}
