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
float right_last_error = 0;

/* PID Controller Gains and Global Variables */
//These values of kp, ki and kd are in the process of being tuned
float right_kp = 3;
float right_ki = 1.2;
float right_kd = 0.05;
//Global variables for P, I, and D terms to retain outside of the main loop
float right_P;
float right_I;
float right_D;


void setup()
{
  Serial.begin(115200);
  float right_beginning_encoder_value = enc2.read(); //this will be at the start of the function
  for (int i = 0; i < 10; i++) {
    tone(BUZZ, 3000+(i*200), 100);
    delay(100);
  }
  
}

void PID_Motor_Control(int right)   //just testing the left in this code for now
{
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, right);
}


void loop()
{
  
  //Current time and encoder readings
  float current_time = micros() ;
  float current_time_s = current_time / 1e6;
  float right_current_encoder_value = -enc2.read();    //read the current encoder value

  /*  Using adjusted encoder value so that there's no issue if it's been running for a long time  */
  float right_adjusted_encoder_value = right_current_encoder_value - right_beginning_encoder_value;  //again for testing the leat wheel
  float right_position_cm = right_adjusted_encoder_value * dist_per_tick;


  // Calculating the change in time
  float delta_time_s = current_time_s - last_time_s;
  //Serial.print(delta_time_s); Serial.print("\t");

  
  /*  Testing the motor controller with the motor 1 (left motor: tired note - left motor is on right side when the robot is upside down)  */
  float right_error_sig = right_position_cm - position_setpoint;

  right_P = right_error_sig;
  right_I = right_I + (right_error_sig * delta_time_s);
  right_D =  (right_error_sig - right_last_error) / delta_time_s;  //check to make sure that delta s isn't too large or too small - might be useless if too long or too short
  right_last_error = right_error_sig;

  float right_motor_control_sig = right_P * right_kp + right_I * right_ki + right_D * right_kd;

  int rightPWM = -right_motor_control_sig;    

  if (rightPWM > MAX_PWM_VALUE) {
    rightPWM = MAX_PWM_VALUE;
  }
  if (rightPWM < 0) {
    rightPWM = 0;
  }

  /* Setting the updated values for the next loop iteratin - Ramping position to match desired velocity, updating the last_time reading */
  float position_increment = test_speed * delta_time_s;  //ramp position to match desired velocity
  float new_position_setpoint = position_setpoint + position_increment;
  position_setpoint = new_position_setpoint;

  last_time_s = current_time_s;
  Serial.print(position_setpoint); Serial.print("\t");  //printing out position setpoint to ensure that it's incrementing


  Serial.print(right_position_cm); Serial.print("\t");
  Serial.print(right_error_sig); Serial.print("\t");

  Serial.print(rightPWM); Serial.print("\t");
  Serial.println();
  PID_Motor_Control(rightPWM);
  
}
