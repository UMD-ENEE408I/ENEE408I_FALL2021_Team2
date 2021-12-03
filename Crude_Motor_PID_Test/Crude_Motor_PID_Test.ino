#include <Encoder.h>

//  pins assigned to the encoder values
const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;

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
const float dist_per_tick = revolution_distance/revolution_ticks;   //cm equivalent of the rotary encoder ticks
const float junction_ticks = 15;  //  514.5 ticks per 15 cm

/*  Variable for the test position - position_setpoint to be incremented with time  */  
float test_speed = 23;    //test speed of 23 cm/s
float position_setpoint = 0;  //set the initial position setpoint to zero
float old_position_setpoint = 0;

//global time variable to keep the old time and reading
float old_time = 0;
int old_encoder_value = 0;

/* PID Controller Gains and Global Variables */
//These values of kp, ki and kd are in the process of being tuned
float kp = 2;
float ki = 0;
float kd = 0;
//Global variables for P, I, and D terms to retain outside of the main loop
float P;
float I;
float D;


void setup() 
{
  Serial.begin(115200);
  float left_beginning_encoder_value = enc1.read(); //this will be at the start of the function
}

void PID_Motor_Control(int left, int right)   //just testing the left in this code for now
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, left);
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, right);
}

/*
void ramp_position(float dt)
{

}
*/

void loop() 
{
  /*  Note: When integrating this into our overall code, need to zero out the */
  Serial.print(position_setpoint); Serial.print("\t");
  //Current time and encoder readings
  float current_time = millis();
  float current_time_s = current_time / 1000;
  float current_encoder_value = enc1.read();    //read the current encoder value


  float adjusted_encoder_value = current_encoder_value - beginning_encoder_value;
  float position_cm = adjusted_encoder_value * dist_per_tick;
  int delta_encoder_value = current_encoder_value - old_encoder_value;  //distance travelled in rotary ticks since the last loop - calculating to help measure why it's overshooting
  
  // Calculating the change in time
  float delta_time_s = current_time_s - old_time_s;
  Serial.print(delta_time); Serial.print("\t");
  
  
  /*  Testing the motor controller with the motor 1 (left motor: tired note - left motor is on right side when the robot is upside down)  */
  error_sig = position_cm - position_setpoint;

  P = error_sig;
  I = I + (error_sig * delta_time_s);
  D =  (error_sig - last_error) / delta_time_s;  //difference
  last_error = error_sig;

  motor_control_sig = P*kp + I*ki + D*kd;

  int leftPWM = PWM_VALUE + motorcontrol; 
  //int rightPWM = PWM_VALUE - motorcontrol;    //commenting out until testing the right motor
  
  /* Ensure that the PWM values are within the 0 to 255 range */
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
    
  /* Setting the updated values for the next loop iteratin - Ramping position to match desired velocity, updating the old_time reading */
  
  float position_increment = test_speed * delta_time_s;  //ramp position to match desired velocity
  float position_setpoint = position_setpoint + position_increment;
  
  old_time_s = current_time_s;
  
}
