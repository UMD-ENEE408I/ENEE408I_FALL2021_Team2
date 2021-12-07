#include <Adafruit_MCP3008.h>
#include <Encoder.h>
#include <Arduino_LSM9DS1.h>

//set chips for spi bus
const unsigned int RF_CS = A4;

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

//motor control pins - potentially pass into motor controller function
const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const int reference_sig = 6;    //the center sensor - 7th sensor (sensors numbers range from 0-12)

//vel values for the motor controller
const float base_vel = 35;    //use for both left and right 

//P term for line follower
float P = 0;
const float kp = 1.6;

//  Motor Controller Variables Start

unsigned int MIN_PWM_VALUE = 0;   //minimum PWM value
unsigned int MAX_PWM_VALUE = 255; //maximum PWM value not to be exceeded
unsigned int PWM_VALUE = 40;

/*  for use when integrated with the overall code - encoder value will not start at 0  */
float left_beginning_encoder_value = 0;
float right_beginning_encoder_value = 0;

/*  Rotary Encoder Values - universal, close enough across all robots */
const float revolution_ticks = 360; //  360 ticks/wheel rotation
const float revolution_distance = 10.5; //  10.5 cm per rotation
const float dist_per_tick = revolution_distance / revolution_ticks; //cm equivalent of the rotary encoder ticks
const float junction_ticks = 15;  //  514.5 ticks per 15 cm

/*  Variable for the test position - position_setpoint to be uniformly incremented with time  */
float left_position_setpoint = 0;   //set the left initial position setpoint to zero
float right_position_setpoint = 0;  //set the right initial position setpoint to zero
float last_position_setpoint = 0;

/* global variables to keep the old readings */
float motor_last_time_s = 0;
float left_last_error = 0;
float right_last_error = 0;

/* PID Controller Gains and Global Variables */
//These values of kp, ki and kd are in the process of being tuned
float left_kp = 3.2;
float left_ki = 1.5;
float left_kd = 0.05;
float right_kp = 3.2;
float right_ki = 1.5;
float right_kd = 0.05;
//Global variables for P, I, and D terms to retain outside of the main loop
float left_P;
float left_I;
float left_D;
float right_P;
float right_I;
float right_D;

//  Motor Controller Variables End

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

  if (gz_deg_adjusted >= -75)    //Ashley's Robot needs this to be 74
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
  if(sensor_sum > 8){  
    if(right_count > 5 && left_count < 5){
     delay(1000);
     Serial.println("right");
     moveForward();
      delay(100);
      moveStop();
      Right_Turn();
    }
    if(right_count < 5 && left_count > 5){
      delay(1000);
      Serial.println("Left");
      moveForward();
      delay(100);
      moveStop();
      Left_Turn();
    }
    if(right_count > 5 && left_count > 5){
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
  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH);
  
  Serial.begin(115200);
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS); 

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
}


// Directly pasted from the motor controller arduino file
void PID_Motor_Control(float left, float right)   //just testing the left in this code for now
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, left);
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, right);
}

/*  motor_type = 1 means left,    motor_type = 2 means right*/
void Motor_Controller(float vel_setpoint_left, float vel_setpoint_right)
{  
    //Current time and encoder readings
    float current_time = micros() ;
    Serial.print(current_time);  Serial.print("\t");
    float current_time_s = current_time / 1e6;
    float left_current_encoder_value = enc1.read();    //read the current encoder value
    
    float right_current_encoder_value = -enc2.read();    //read the current encoder value
  
    /*  Using adjusted encoder value so that there's no issue if it's been running for a long time  */
    float left_adjusted_encoder_value = left_current_encoder_value - left_beginning_encoder_value;  //again for testing the leat wheel
    float left_position_cm = left_adjusted_encoder_value * dist_per_tick;
  
    float right_adjusted_encoder_value = right_current_encoder_value - right_beginning_encoder_value;  //again for testing the leat wheel
    float right_position_cm = right_adjusted_encoder_value * dist_per_tick;
  
  
    /* Calculating the change in time */
    float delta_time_s = current_time_s - motor_last_time_s;
  
    
    /*  Testing the motor controller with the motor 1 (left motor: tired note - left motor is on right side when the robot is upside down)  */
    float left_error_sig = left_position_cm - left_position_setpoint;
    float right_error_sig = right_position_cm - right_position_setpoint;
  
    left_P = left_error_sig;
    left_I = left_I + (left_error_sig * delta_time_s);
    left_D =  (left_error_sig - left_last_error) / delta_time_s;  //check to make sure that delta s isn't too large or too small - might be useless if too long or too short
    left_last_error = left_error_sig;
  
    right_P = right_error_sig;
    right_I = right_I + (right_error_sig * delta_time_s);
    right_D =  (right_error_sig - right_last_error) / delta_time_s;  //check to make sure that delta s isn't too large or too small - might be useless if too long or too short
    right_last_error = right_error_sig;
  
    float left_motor_control_sig = left_P * left_kp + left_I * left_ki + left_D * left_kd;
    float right_motor_control_sig = right_P * right_kp + right_I * right_ki + right_D * right_kd;
  
    float leftPWM = -left_motor_control_sig;
    float rightPWM = -right_motor_control_sig;   
  
    /* Ensure that the PWM values are within the 0 to 255 range */
    if (leftPWM > MAX_PWM_VALUE) {
      leftPWM = MAX_PWM_VALUE;
    }
    if (leftPWM < 0) {
      leftPWM = 0;
    }
  
    if (rightPWM > MAX_PWM_VALUE) {
      rightPWM = MAX_PWM_VALUE;
    }
    if (rightPWM < 0) {
      rightPWM = 0;
    }
  
    /* Setting the updated values for the next loop iteratin - Ramping position to match desired velocity, updating the last_time reading */
    float left_position_increment = vel_setpoint_left * delta_time_s;  //ramp position to match desired left velocity
    float right_position_increment = vel_setpoint_right * delta_time_s; //ramp position to match desired right velocity

    float left_new_position_setpoint = left_position_setpoint + left_position_increment;
    float right_new_position_setpoint = right_position_setpoint + right_position_increment;
    left_position_setpoint = left_new_position_setpoint;
    right_position_setpoint = right_new_position_setpoint;
  
    motor_last_time_s = current_time_s;
    Serial.print(left_position_setpoint); Serial.print("\t");  //printing out position setpoint to ensure that it's incrementing
    Serial.print(left_position_cm); Serial.print("\t");
    Serial.print(left_error_sig); Serial.print("\t");
    Serial.print(leftPWM); Serial.print("\t");
    
    
    Serial.print(right_position_setpoint); Serial.print("\t");  //printing out position setpoint to ensure that it's incrementing
    Serial.print(right_position_cm); Serial.print("\t");
    Serial.print(right_error_sig); Serial.print("\t");
    Serial.print(rightPWM); Serial.print("\t");
    Serial.println();
    
    PID_Motor_Control(leftPWM, rightPWM);
  
}//end of motor controller function


void PID_Motor_Controller(float left, float right)   //just testing the left in this code for now
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, left);
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, right);
}

void loop() 
{
  //initializing vaiables
  int adc_buf[13];
  int bin_buf[13];  // binary array for sensor array values
 
  //cutoff values for binary conversion - will vary for each robot
  int cutoff[13] = {690,690,690,690,690,690,690,690,690,690,690,690,690}; 

  float vel_control_sig = 0;
  float sensor_sum = 0; // sum of sensor positions reading the line
  float sensor_count = 0; // count of the sensors reading the line
  int right_sensor_count = 0; //count of the right sensors (positive error) reading the line
  int left_sensor_count = 0;  //count of the left sensors (negative error) reading the line
  int junction_case = 1;   //Turning cases - 1 = straight, 2 = right turn, 3 = left turn, 4 = t junction T, 5 = right t junction |- , 6 = left t junction -| , 7 = fourway,  8 = dead end
  int same_junction_count = 0;  //check to make sure that not turning around in a circle in junction cases 2 and 3
  float center_pos;   // center of line sensor position 
  float main_error_sig;    // error signal 


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
  
  // Calculating sensor average - need this for case that is not a junction
  if (sensor_count > 0)
  {
     center_pos = sensor_sum / sensor_count;
  }
  else
  {
     center_pos = - 1;
  }

  main_error_sig = reference_sig - center_pos;

  
  // print out adc values, center of line position and error signal
  for (int i = 0; i < 13; i++) 
  {
    Serial.print(bin_buf[i]); Serial.print("\t");
  }

  Serial.print(center_pos); Serial.print("\t");
  Serial.print(main_error_sig); Serial.print("\t");
  
  // basic pid line line following when on the line
  if (main_error_sig != 7)
  {
    P = main_error_sig;
    vel_control_sig = P*kp;
  }
  
  //Check the motor control signal being fed into the wheels    - instead of PWM_value, this will become the velocity setpoints

  //calculate left and right PWM values - change this to desired velocity/position setpoint
  float left_vel = base_vel + vel_control_sig; 
  float right_vel = base_vel - vel_control_sig;

  Serial.print(left_vel); Serial.print("\t");
  Serial.print(right_vel); Serial.print("\t");
  
  //stop PID control if the robot is off the line
  if (main_error_sig == 7) //stop the robot when it's off the line
  {
    moveStop();
  }
  else
  {
    
    Serial.print(micros());
    PID_Motor_Controller(left_vel,right_vel);
    Junction(sensor_sum, left_sensor_count, right_sensor_count);
  }
  Serial.println();
}
