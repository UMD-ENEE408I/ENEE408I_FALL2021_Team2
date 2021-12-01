#include <Arduino_LSM9DS1.h>

float gx, gy, gz;
float I_gz;

float gz_offset = 0;
float gz_offset_s = 0;
long gz_time = 0;
float gz_deg = 0;   //degrees in the z direction
float gz_vel_old = 0;   //set initial gz value to 0 until it's set in the loop

//motor control pins
const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int MIN_PWM_VALUE = 0;   //minimum PWM value 
const unsigned int MAX_PWM_VALUE = 255; //maximum PWM value not to be exceeded
const unsigned int PWM_VALUE = 60;      //Base moving speed for the robot (for testing purposes)

void M1_forward() 
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM_VALUE);
}

void M2_forward() 
{
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM_VALUE);
}

void M1_stop() 
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}


void M2_stop() 
{
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}

void turnRight()
{
    M1_forward();
    M2_stop();
}

void turnLeft()
{
    M1_stop();
    M2_forward();
}

void setup() 
{
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");
  
  float sum_gz = 0;

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");
  
  unsigned long StartTime = millis();

  for (int i = 0; i<238 ;i++)
  {
     if (IMU.gyroscopeAvailable()) 
     {
        IMU.readGyroscope(gx, gy, gz);
     }
     sum_gz = sum_gz + gz;
     delay(8.4);  //delay of 8.4 ms between readings
  }
  unsigned long EndTime = millis();
  gz_offset = sum_gz/(238*8.4);   //The correction for gz drift
  gz_offset_s = gz_offset/1000;
  
  unsigned long EllapsedTime = EndTime - StartTime;
  /*
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
  Serial.println(gz_offset);
  Serial.println();
  */
}

void loop() 
{

 //Gyroscope angle calculation
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
   unsigned long newgyroTime = millis();
   float deltaTime = newgyroTime - gz_time; //The delta time since the last 
   float deltaTimeS = deltaTime/1000;
   float new_gz_vel;
   float avg_gz_vel;

 // Serial.print(newTime);Serial.print("\t");
 // Serial.print(deltaTime);Serial.print("\t");
 // Serial.print(deltaTimeS);Serial.print("\t");

//finding the average angular velocity

  if (gz_time != 0)
  {
    avg_gz_vel = ((gz - gz_offset) + gz_vel_old)/2;  //averaging the current gz velocity minus
    gz_deg = gz_deg + ((avg_gz_vel * deltaTimeS)); //integrating the average ang vel 
    Serial.print(gz_deg);
    Serial.println();
  }

  gz_time = newgyroTime;
  gz_vel_old = gz;

  //Testing Turn Right
  if (gz_deg > -80)
  {
    turnRight(); 
  }
  
}
