#include <Adafruit_MCP3008.h>
#include <Encoder.h>
#include <Arduino_LSM9DS1.h>

//set chips for spi bus
const unsigned int RF_CS = A4;

//intersection verification
int Intersection_left = 0;
int Intersection_right = 0;

//Intersection
int Intersections_Taken[50];
int Intersection_Count = 0;

//rotary encoder pins
const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;

//Rotary encoders
Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);

//Analog to digital converters
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

//sensor array - analog to digital converters
const unsigned int ADC_1_CS = A3;
const unsigned int ADC_2_CS = A2;

//motor control pins
const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

//
const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const int reference_sig = 6;    //the center sensor - 7th sensor (sensors numbers range from 0-12)

const unsigned int MIN_PWM_VALUE = 0;   //minimum PWM value 
const unsigned int MAX_PWM_VALUE = 255; //maximum PWM value not to be exceeded
const unsigned int PWM_VALUE = 35;      //Base moving speed for the robot (for testing purposes)

float last_error = 0;

//setting gain values
float kp = 2;
float ki = 0;
float kd = 0;
float P;
float I;
float D;
float t_old = 0;  

// Basic Motor Control Commands

void PID_Motor_Control(int left, int right)
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, left);
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, right);
}

void M1_forward() 
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM_VALUE);
}

void M1_stop() 
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}

void M1_backward() 
{
  analogWrite(M1_IN_1, PWM_VALUE);
  analogWrite(M1_IN_2, 0);
}

void M2_forward() 
{
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM_VALUE);
}

void M2_stop() 
{
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}

void M2_backward() 
{
  analogWrite(M2_IN_1, PWM_VALUE);
  analogWrite(M2_IN_2, 0);
}

void moveStop()
{
    M1_stop();
    M2_stop();
}

void moveForward()
{
    M1_forward();
    M2_forward();
}    

void moveBackward()
{
    M1_backward();
    M2_backward();
}

void turnRight()
{
    M1_forward();
    M2_stop();
}

void sharp_turnRight()
{
    M1_forward();
    M2_backward();
}

void turnLeft()
{
    M1_stop();
    M2_forward();
}

void sharp_turnLeft()
{
    M1_backward();
    M2_forward();
}

//turn functions
void Left_Turn(){
  
long gz_time = 0;
float sum_gz = 0;
float gx, gy, gz;
float I_gz;
float gz_offset = 0;
float gz_offset_s = 0;
float gz_deg = 0;   //degrees in the z direction
float gz_deg_adjusted = 0;
float gz_vel_old = 0;   //set initial gz value to 0 until it's set in the loop
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

int turn = 1;

while(turn > 0){
 //Gyroscope angle calculation
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
   unsigned long newgyroTime = millis();
   float deltaTime = newgyroTime - gz_time; //The delta time since the last 
   float deltaTimeS = deltaTime/1000;
   float new_gz_vel;
   float avg_gz_vel;

  if (gz_time != 0)
  {
    avg_gz_vel = ((gz - gz_offset) + gz_vel_old)/2;  //averaging the current gz velocity minus
    gz_deg = gz_deg + ((avg_gz_vel * deltaTimeS)); //integrating the average ang vel 
    gz_deg_adjusted = gz_deg * 90/80;
  }

  gz_time = newgyroTime;
  gz_vel_old = gz;

  if (gz_deg_adjusted <= 75)    //Ashley's Robot needs this to be 74
  {
    turnLeft();
  }
  else
  {
    moveStop();
    return;
  }
 }
} 

void Right_Turn(){

long gz_time = 0;
float sum_gz = 0;
float gx, gy, gz;
float I_gz;
float gz_offset = 0;
float gz_offset_s = 0;
float gz_deg = 0;   //degrees in the z direction
float gz_deg_adjusted = 0;
float gz_vel_old = 0;   //set initial gz value to 0 until it's set in the loop

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
  
int turn = 1;

while(turn > 0){
 //Gyroscope angle calculation
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
   unsigned long newgyroTime = millis();
   float deltaTime = newgyroTime - gz_time; //The delta time since the last 
   float deltaTimeS = deltaTime/1000;
   float new_gz_vel;
   float avg_gz_vel;

  if (gz_time != 0)
  {
    avg_gz_vel = ((gz - gz_offset) + gz_vel_old)/2;  //averaging the current gz velocity minus
    gz_deg = gz_deg + ((avg_gz_vel * deltaTimeS)); //integrating the average ang vel 
    gz_deg_adjusted = gz_deg * 90/80;
  }

  gz_time = newgyroTime;
  gz_vel_old = gz;

  if (gz_deg_adjusted <= -75)    //Ashley's Robot needs this to be 74
  {
    turnRight();
  }
  else
  {
    moveStop();
    return;
  }
 }
} 

void Turn_180(){
  
long gz_time = 0;
float sum_gz = 0;
float gx, gy, gz;
float I_gz;
float gz_offset = 0;
float gz_offset_s = 0;
float gz_deg = 0;   //degrees in the z direction
float gz_deg_adjusted = 0;
float gz_vel_old = 0;   //set initial gz value to 0 until it's set in the loop
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

int turn = 1;

while(turn > 0){
 //Gyroscope angle calculation
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
   unsigned long newgyroTime = millis();
   float deltaTime = newgyroTime - gz_time; //The delta time since the last 
   float deltaTimeS = deltaTime/1000;
   float new_gz_vel;
   float avg_gz_vel;

  if (gz_time != 0)
  {
    avg_gz_vel = ((gz - gz_offset) + gz_vel_old)/2;  //averaging the current gz velocity minus
    gz_deg = gz_deg + ((avg_gz_vel * deltaTimeS)); //integrating the average ang vel 
    gz_deg_adjusted = gz_deg * 90/80;
  }

  gz_time = newgyroTime;
  gz_vel_old = gz;

  if (gz_deg_adjusted <= 155)    //Ashley's Robot needs this to be 74
  {
    turnLeft();
  }
  else
  {
    moveStop();
    return;
  }
 }
} 

void Junction(int sensor_sum, int left_count, int right_count){
  if(sensor_sum > 4){  
    if(right_count > 3 && left_count < 3){
     delay(1000);
     Serial.println("right");
     moveForward();
      delay(100);
      moveStop();
      Right_Turn();
    }
    if(right_count < 3 && left_count > 3){
      delay(1000);
      Serial.println("Left");
      moveForward();
      delay(100);
      moveStop();
      Left_Turn();
    }
    if(right_count > 3 && left_count > 3){
      delay(1000);
      Serial.println("T Junction");
      moveForward();
      delay(100);
      moveStop();
      Left_Turn();    
    }
  }/*
  if(sensor_sum < 1){
    delay(1000);
    Serial.println("dead end");
    Turn_180();
    delay(1000);
  }*/
}

void setup()
{
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.begin(115200);
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS); 

  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH);
}

//Put center of line algorithm into its own function
void CenterOfLine()
{
  
}

// A separate controller for the motor control based on ramp position input
void MotorController()
{

}

void loop() 
{
  //initializing vaiables
  int adc_buf[13];
  int bin_buf[13];  // binary array for sensor array values
 
  //cutoff values for binary conversion - will vary for each robot
  int cutoff[13] = {680,600,600,600,600,600,600,600,600,600,600,680}; 
  
  int sensor_sum = 0; // sum of sensor positions reading the line
  int sensor_count = 0; // count of the sensors reading the line
  int right_sensor_count = 0; //count of the right sensors (positive error) reading the line
  int left_sensor_count = 0;  //count of the left sensors (negative error) reading the line
  int junction_case = 1;   //Turning cases - 1 = straight, 2 = right turn, 3 = left turn, 4 = t junction T, 5 = right t junction |- , 6 = left t junction -| , 7 = fourway,  8 = dead end
  int same_junction_count = 0;  //check to make sure that not turning around in a circle in junction cases 2 and 3
  float center_pos;   // center of line sensor position 
  int error_sig;    // error signal 
  float motorcontrol;
  

  int dead_end = 0;
  
  // Reading sensor array values
  for (int i = 0, j = 0; i < 13; i = i + 2) 
  {
    adc_buf[i] = adc1.readADC(j);
    if (i < 12)
    {
       adc_buf[i+1] = adc2.readADC(j);
    }
    j++;
  }

  // Convert sensor values to binary, increment sensor count and add sensor positions that are reading the line
  for (int i = 0; i<13 ; i++)
  {
    if (adc_buf[i]<cutoff[i]) //may need to adjust cutoffs for the white/black values
    { 
      bin_buf[i] = 1;    //binary value of 1 for sensors that are reading the line
      sensor_sum = sensor_sum + i;
      sensor_count++;
      //increment right or left side sensor count depending on sensors reading the line
      if (i<6)
      {
        right_sensor_count++;
        
      }
      if (i>6)
      {
        left_sensor_count++;  
      }
    }
    else
    {
      bin_buf[i] = 0;
    }
  }

  
  if(right_sensor_count > 3){
    Intersection_right++;
  }

  if(left_sensor_count > 3){
    Intersection_left++;
  }
  if(Intersection_left > 2 || Intersection_right > 2){    
  Junction(sensor_sum, left_sensor_count, right_sensor_count);
  Intersection_left = 0;
  Intersection_right = 0;  
  }
  
/*
//Added starting here
//Junction Key: 1 = right turn, 2 = right turn junction |- , 3 = left turn, 4 = left turn junction -| , 5 = T junction, 6 = + junction, 7 - dead end
  if(left_sensor_count > 2 && right_sensor_count < 2){
    if(adc_buf[6] > cutoff[6]){
      //Serial.println("Left");
      Left_Turn();
      Intersection = 3;

   //   Intersections_Taken[Intersection_Count] = Intersection;
   //   Intersection_Count++;
    }
    if(adc_buf[6] < cutoff[6]){
   //   Serial.println(" left turn junction -| ");
      Intersection = 4;
   //   Intersections_Taken[Intersection_Count] = Intersection;
   //   Intersection_Count++;
    }
  }

  if (right_sensor_count > 2 && left_sensor_count < 2){
    if(adc_buf[6] > cutoff[6]){
      //Serial.println("Right");
      Right_Turn();
      Intersection = 1;
   //   Intersections_Taken[Intersection_Count] = Intersection;
   //   Intersection_Count++;
    }
    if(adc_buf[6] < cutoff[6]){
     // Serial.println(" right turn junction |- ");
      Intersection = 2;
   //   Intersections_Taken[Intersection_Count] = Intersection;
   //   Intersection_Count++;
    }
  }

  if(right_sensor_count > 2 && left_sensor_count > 2){
    if(adc_buf[6] > cutoff[6]){
     // Serial.println("T Junction");
      Left_Turn();
      Intersection = 5;
   //   Intersections_Taken[Intersection_Count] = Intersection;
   //   Intersection_Count++;
    }
    if(adc_buf[6] < cutoff[6]){

      Intersection = 6;
   //   Intersections_Taken[Intersection_Count] = Intersection;
   //   Intersection_Count++;
    }
  }

  //dead end case
  if(sensor_count < 1){
    Intersection = 7;
 //   Turn_180();
  //  Intersections_Taken[Intersection_Count] = Intersection;
   // Intersection_Count++;
  }
  
 // for(int i = 0; i<5; i++){
 //   Serial.println(Intersections_Taken[i]);
 // }
  

//Added up until here
*/


  
  // Calculating sensor average - need this for case that is not a junction - straight line - should be the majority of cases)
  if (sensor_count > 0)
  {
     center_pos = sensor_sum / sensor_count;
  }
  else
  {
     center_pos = - 1;
  }

  error_sig = reference_sig - center_pos;
  
  // basic pid line line following when on the line
  if (error_sig != 7)
  {
    P = error_sig;
    motorcontrol = P*kp;
  }

  
  //Check the motor control signal being fed into the wheels

  //calculate left and right PWM values - change this to desired velocity/position setpoint
  //to be replaced with motor controller code/function
  int leftPWM = PWM_VALUE + motorcontrol; 
  int rightPWM = PWM_VALUE - motorcontrol;

  //make sure PWM values are within the 0 to 25 range
  if (leftPWM > MAX_PWM_VALUE) {
    leftPWM = MAX_PWM_VALUE;
  }
  if (rightPWM > MAX_PWM_VALUE) {
    rightPWM = MAX_PWM_VALUE;
  }
  if (leftPWM < 0) {
    leftPWM = 0;
  }
  if (rightPWM < 0) {
    rightPWM = 0;
  } 
  
  //stop PID control if the robot is off the line
  if (error_sig == 7) //stop the robot when it's off the line
  {
    leftPWM = 35;     //Without doing this, the robot 
    rightPWM = 35;    //
    motorcontrol = 0; //
    moveStop();
  }
  else
  {
    PID_Motor_Control(leftPWM,rightPWM);
  }
  
  //delay(50);    //delay of - may make sense to use RTOS
}
