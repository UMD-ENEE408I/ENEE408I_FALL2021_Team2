#include <Adafruit_MCP3008.h>
#include <Encoder.h>

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

const int reference_sig = 6;

const unsigned int MIN_PWM_VALUE = 0;   //minimum PWM value 
const unsigned int MAX_PWM_VALUE = 255; //maximum PWM value not to be exceeded
const unsigned int PWM_VALUE = 60;      //Base moving speed for the robot (for testing purposes)

float last_error = 0;

float kp = 1.4;
float ki = 0.08;
float kd = 0;
int P;
int I;
int D;


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

void setup()
{
  Serial.begin(115200);
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS); 
}

void CenterOfLiner()
{
  
}

void loop() 
{
  //initializing vaiables
  int adc_buf[13];
  int bin_buf[13];  // binary array for sensor array values
 
  //cutoff values for binary conversion - will vary for each robot
  int cutoff[13] = {600,600,600,600,600,600,600,600,600,600,600,650,700}; 
  
  int sensor_sum = 0; // sum of sensor positions reading the line
  int sensor_count = 0; // count of the sensors reading the line
  int right_sensor_count = 0; //count of the right sensors (positive error) reading the line
  int left_sensor_count = 0;  //count of the left sensors (negative error) reading the line
  int junction_case = 1;   //Turning cases - 1 = straight, 2 = right turn, 3 = left turn, 4 = t junction T, 5 = right t junction |- , 6 = left t junction -| , 7 = dead end
  int same_junction count = 0;  //check to make sure that not turning around in a circle in junction cases 2 and 3
  int center_pos;   // center of line sensor position 
  int error_sig;    // error signal 
  float motorcontrol;


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
        left_sensor_count++;
      }
      if (i>6)
      {
        right_sensor_count++;  
      }
    }
    else
    {
      bin_buf[i] = 0;
    }
  }
  
  // Calculating sensor average based 
  if (sensor_count > 0)
  {
     center_pos = sensor_sum / sensor_count;
  }
  else
  {
     center_pos = - 1;
  }

  error_sig = reference_sig - center_pos;

  //start counting time for differentiation and integration values 
  int t_start = micros();
  // print out adc values, center of line position and error signal
  for (int i = 0; i < 13; i++) 
  {
    Serial.print(bin_buf[i]); Serial.print("\t");
  }

  Serial.print(center_pos); Serial.print("\t");
  Serial.print(error_sig); Serial.print("\t");
  
  // basic pid line line following when on the line
  if (error_sig != 7)
  {
    P = error_sig;
    I = I + error_sig;
    D =  (error_sig - last_error);  //difference
    last_error = error_sig;
    motorcontrol = P*kp + I*ki + D*kd;
  }
  else
  {
    I = 0;
    D = 0;
  }
  
  Serial.print(motorcontrol); Serial.print("\t");
  
  //Check the motor control signal being fed into the wheels

  //calculate left and right PWM values
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
    leftPWM = 60;
    rightPWM = 60;
    motorcontrol = 0;
    moveStop();
  }
  else
  {
    PID_Motor_Control(leftPWM,rightPWM);
  }
  Serial.print(motorcontrol); Serial.print("\t");
  Serial.print(leftPWM); Serial.print("\t");
  Serial.print(rightPWM); Serial.print("\t");
  Serial.println();

  t_end = t_start;
  
  delay(50);    //delay of - may make sense to use RTOS
}
