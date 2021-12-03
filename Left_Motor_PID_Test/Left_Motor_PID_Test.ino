#include <Adafruit_MCP3008.h>
#include <Encoder.h>

//  pins assigned to the encoder values
const unsigned int M1_ENC_A = 6;
const unsigned int M2_ENC_A = 8;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_B = 9;

//add it in for buzzer
const unsigned int BUZZ = 10;

Encoder enc1(M1_ENC_A, M1_ENC_B); //Encoder for Motor 1 (left)
Encoder enc2(M2_ENC_A, M2_ENC_B); //Encoder for Motor 2 (right)

//  motor control pins
const unsigned int M1_IN_1 = 2;   //motor 1 backward
const unsigned int M1_IN_2 = 3;   //motor 1 forward
const unsigned int M2_IN_1 = 5;   //motor 2 backward
const unsigned int M2_IN_2 = 4;   //motor 2 forward

const unsigned int MIN_PWM_VALUE = 0;   //minimum PWM value
const unsigned int MAX_PWM_VALUE = 255; //maximum PWM value not to be exceeded
const unsigned int PWM_VALUE = 60;      //Base moving speed for the robot (for testing purposes)

/*  for use when integrated with the overall code - encoder value will not start at 0  */
const int left_beginning_encoder_value = 0;
const int right_beginning_encoder_value = 0;

/*  Rotary Encoder Values  */
const float revolution_ticks = 360; //  360 ticks/wheel rotation
const float revolution_distance = 10.5; //  10.5 cm per rotation
const float dist_per_tick = revolution_distance / revolution_ticks; //cm equivalent of the rotary encoder ticks
const float junction_ticks = 15;  //  514.5 ticks per 15 cm

/*  Variable for the test position - position_setpoint to be incremented with time  */
float test_speed = 23;    //test speed of 23 cm/s
float position_setpoint = 0;  //set the initial position setpoint to zero
float last_position_setpoint = 0;

/* global variables to keep the old readings */
float last_time_s = 0;
int last_encoder_value = 0;
float last_error = 0;

/* PID Controller Gains and Global Variables */
//These values of kp, ki and kd are in the process of being tuned
float kp = 3;
float ki = 1.2;
float kd = 0.05;
//Global variables for P, I, and D terms to retain outside of the main loop
float P;
float I;
float D;


void setup()
{
  Serial.begin(115200);
  float left_beginning_encoder_value = enc1.read(); //this will be at the start of the function
  //float right_beginning_encoder_value = enc1.read(); //this will be at the start of the function
  for (int i = 0; i < 10; i++) {
    tone(BUZZ, 3000+(i*200), 100);
    delay(100);
  }
  
}

void PID_Motor_Control(int left, int right)   //just testing the left in this code for now
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, left);
  //analogWrite(M2_IN_1, 0);
  //analogWrite(M2_IN_2, right);
}

/*
  void ramp_position(float dt)
  {

  }
*/

void loop()
{
  
  //Current time and encoder readings
  float current_time = micros() ;
  float current_time_s = current_time / 1e6;
  float current_encoder_value = enc1.read();    //read the current encoder value

  /*  Using adjusted encoder value so that there's no issue if it's been running for a long time  */
  float adjusted_encoder_value = current_encoder_value - left_beginning_encoder_value;  //again for testing the leat wheel
  float position_cm = adjusted_encoder_value * dist_per_tick;
  int delta_encoder_value = current_encoder_value - last_encoder_value;  //distance travelled in rotary ticks since the last loop - calculating to help measure why it's overshooting

  // Calculating the change in time
  float delta_time_s = current_time_s - last_time_s;
  //Serial.print(delta_time_s); Serial.print("\t");

  
  /*  Testing the motor controller with the motor 1 (left motor: tired note - left motor is on right side when the robot is upside down)  */
  float error_sig = position_cm - position_setpoint;

  P = error_sig;
  I = I + (error_sig * delta_time_s);
  D =  (error_sig - last_error) / delta_time_s;  //check to make sure that delta s isn't too large or too small - might be useless if too long or too short
  last_error = error_sig;

  float motor_control_sig = P * kp + I * ki + D * kd;

  int leftPWM = -motor_control_sig;
  //int rightPWM = PWM_VALUE - motor_control_sig;    //commenting out until testing the right motor

  /* Ensure that the PWM values are within the 0 to 255 range */
  if (leftPWM > MAX_PWM_VALUE) {
    leftPWM = MAX_PWM_VALUE;
  }
  //comment out while testing the left case
  //if (rightPWM > MAX_PWM_VALUE) {
  //  rightPWM = MAX_PWM_VALUE;
  //}
  if (leftPWM < 0) {
    leftPWM = 0;
  }
  //comment out while testing the left case
  //if (rightPWM < 0) {
  //  rightPWM = 0;
  //}

  /* Setting the updated values for the next loop iteratin - Ramping position to match desired velocity, updating the last_time reading */
  float position_increment = test_speed * delta_time_s;  //ramp position to match desired velocity
  float new_position_setpoint = position_setpoint + position_increment;
  position_setpoint = new_position_setpoint;

  last_time_s = current_time_s;
  Serial.print(position_setpoint); Serial.print("\t");  //printing out position setpoint to ensure that it's incrementing


  Serial.print(position_cm); Serial.print("\t");
  Serial.print(error_sig); Serial.print("\t");

  Serial.print(leftPWM); Serial.print("\t");
  Serial.println();
  PID_Motor_Control(leftPWM, 0);
  
}
