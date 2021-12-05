/*  The file with the PID motor controllers  */
/*  Turn into header file for the main code - needs to be saved in the same folder that the main code is being run from  */

#include <Adafruit_MCP3008.h>
#include <Encoder.h>

//  pins assigned to the encoder values
const unsigned int M1_ENC_A = 6;
const unsigned int M2_ENC_A = 8;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_B = 9;

//pin for buzzer
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

/*  for use when integrated with the overall code - encoder value will not start at 0  */
const int beginning_encoder_value = 0;

/*  Rotary Encoder Values - universal, close enough across all robots */
const float revolution_ticks = 360; //  360 ticks/wheel rotation
const float revolution_distance = 10.5; //  10.5 cm per rotation
const float dist_per_tick = revolution_distance / revolution_ticks; //cm equivalent of the rotary encoder ticks
const float junction_ticks = 15;  //  514.5 ticks per 15 cm

/*  Variable for the test position - position_setpoint to be uniformly incremented with time  */
float test_speed = 20;    //  test speed for this function - Brian wants the robot to run around 20 cm/s for mapping and turning purposes
float position_setpoint = 0;  //set the initial position setpoint to zero
float last_position_setpoint = 0;

/* global variables to keep the old readings */
float motor_last_time_s = 0;
float last_error = 0;

/* PID Controller Gains and Global Variables - tuning works well for the 10 cm/s to 80 cm/s range, which is what we'll primarily be operating in  */
float kp = 3;
float ki = 1.2;
float kd = 0;

/*Global variables for P, I, and D terms to retain outside of the main function */
float P;
float I;
float D;

void setup() 
{
  Serial.begin(115200);
}

void PID_Motor_Control(float left, float right)   //just testing the left in this code for now
{
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, left);
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, right);
}

/*  motor_type = 1 means left,    motor_type = 2 means right*/
void Motor_Controller(float vel_setpoint, int motor_type)
{

  //Setup values - get currrent encoder value to ensure no issues from starting later in the encoder cycle
  if (motor_type == 1)
  {
    Serial.print('Motor type is 1');
    float beginning_encoder_value = enc1.read();
    while(1)    //change this to a specific error threshold as an interrupt so this doesn't infinitely loop, or a break statement once the error threshold is reached
    {
      float current_time = micros() ;
      float current_time_s = current_time / 1e6;
      float current_encoder_value = enc1.read();    //read the current encoder value
    
      /*  Using adjusted encoder value so that there's no issue if it's been running for a long time  */
      float adjusted_encoder_value = current_encoder_value - beginning_encoder_value;  //again for testing the leat wheel
      float position_cm = adjusted_encoder_value * dist_per_tick;
    
    
      /* Calculating the change in time */
      float delta_time_s = current_time_s - motor_last_time_s;
    
      
      /*  Testing the motor controller with the motor 1 (left motor: tired note - left motor is on right side when the robot is upside down)  */
      float error_sig = position_cm - position_setpoint;
    
      P = error_sig;
      I = I + (error_sig * delta_time_s);
      D =  (error_sig - last_error) / delta_time_s;  //check to make sure that delta s isn't too large or too small - might be useless if too long or too short
      last_error = error_sig;
    
      float motor_control_sig = P * kp + I * ki + D *  kd;
    
      float PWM = -motor_control_sig;
    
      /* Ensure that the PWM values are within the 0 to 255 range */
      if (PWM > MAX_PWM_VALUE) {
        PWM = MAX_PWM_VALUE;
      }
      if (PWM < MIN_PWM_VALUE) {
        PWM = MIN_PWM_VALUE;
      }
    
      /* Setting the updated values for the next loop iteratin - Ramping position to match desired velocity, updating the last_time reading */
      float position_increment = vel_setpoint * delta_time_s;  //ramp position to match desired velocity
      float new_position_setpoint = position_setpoint + position_increment;
      position_setpoint = new_position_setpoint;
    
      motor_last_time_s = current_time_s;
      /* Serial Monitor */
      Serial.print(position_setpoint); Serial.print("\t");  //printing out position setpoint to ensure that it's incrementing
      Serial.print(position_cm); Serial.print("\t");
      Serial.print(error_sig); Serial.print("\t");
      Serial.print(PWM); Serial.print("\t");
      Serial.println();
      
      PID_Motor_Control(PWM, 0);    //send motor signal to the left PWM
    } //end of main motor 1 while loop
    
  } //end of motor type 1 if statement
  else if (motor_type == 2)
  {
    Serial.print('Motor type is 2');
    float beginning_encoder_value = -enc2.read();
 
    while(1)  //change this to a specific error threshold as an interrupt so this doesn't infinitely loop, or a break statement once the error threshold is reached
    {
      float current_time = micros() ;
      float current_time_s = current_time / 1e6;
      float current_encoder_value = -enc2.read();    //read the current encoder value
    
      /*  Using adjusted encoder value so that there's no issue if it's been running for a long time  */
      float adjusted_encoder_value = current_encoder_value - beginning_encoder_value;  //again for testing the leat wheel
      float position_cm = adjusted_encoder_value * dist_per_tick;
    
    
      /* Calculating the change in time */
      float delta_time_s = current_time_s - motor_last_time_s;
    
      
      /*  Testing the motor controller with the motor 1 (left motor: tired note - left motor is on right side when the robot is upside down)  */
      float error_sig = position_cm - position_setpoint;
    
      P = error_sig;
      I = I + (error_sig * delta_time_s);
      D =  (error_sig - last_error) / delta_time_s;  //check to make sure that delta s isn't too large or too small - might be useless if too long or too short
      last_error = error_sig;
    
      float motor_control_sig = P * kp + I * ki + D *  kd;
    
      float PWM = -motor_control_sig;
    
      /* Ensure that the PWM values are within the 0 to 255 range */
      if (PWM > MAX_PWM_VALUE) {
        PWM = MAX_PWM_VALUE;
      }
      if (PWM < MIN_PWM_VALUE) {
        PWM = MIN_PWM_VALUE;
      }
    
      /* Setting the updated values for the next loop iteratin - Ramping position to match desired velocity, updating the last_time reading */
      float position_increment = vel_setpoint * delta_time_s;  //ramp position to match desired velocity
      float new_position_setpoint = position_setpoint + position_increment;
      position_setpoint = new_position_setpoint;
    
      motor_last_time_s = current_time_s;
      /* Serial Monitor */
      Serial.print(position_setpoint); Serial.print("\t");  //printing out position setpoint to ensure that it's incrementing
      Serial.print(position_cm); Serial.print("\t");
      Serial.print(error_sig); Serial.print("\t");
      Serial.print(PWM); Serial.print("\t");
      Serial.println();
      
      PID_Motor_Control(0, PWM);   //send motor signal to the right PWM
    }   //end of main motor 2 while loop
  } // end of motor type 2 if statement
  else
  {
    Serial.print('An invalid value for the motor type was input');
  }

  //check for motor type - if right motor, current encoder value must be negative since mounted in opposite 

  //when exiting function, reset the global variables
}

void loop() {
  //Motor_Controller(23,1);   //testing left motor
  //Motor_Controller(23,2);   //testing right motor
}
