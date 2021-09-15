#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = A3;
const unsigned int ADC_2_CS = A2;

const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int PWM_VALUE = 40;

void M1_backward() {
  analogWrite(M1_IN_1, PWM_VALUE);
  analogWrite(M1_IN_2, 0);
}

void M1_forward() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM_VALUE);
}

void M1_stop() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}

void M2_backward() {
  analogWrite(M2_IN_1, PWM_VALUE);
  analogWrite(M2_IN_2, 0);
}

void M2_forward() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM_VALUE);
}

void M2_stop() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}
void setup(){
  Serial.begin(115200);
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS); 
}
void loop() {
  int adc1_buf[8];
  int adc2_buf[8];
  int adc_buf[14];
  int bin_buf[14];
  int temp = 0;
  int qq;
  int t_start = micros();
  int j = 0;
  int q = 1;
  int h = 0;
  for (int i = 0; i < 8; i++) {
    adc_buf[j] = adc1.readADC(i);
    adc_buf[q] = adc2.readADC(i);
    j = j+2;
    q = q+2;
  }
  for(int i = 0; i < 13; i++){
    if(adc_buf[i] < 600){
      bin_buf[i] = 1; 
      temp = temp+i;
    }
    if(adc_buf[i] > 600){
      bin_buf[i] = 0;
    }
  }

  for (int i = 0; i < 15; i++) {
    //Serial.print(adc1_buf[i]); Serial.print("\t");
    //Serial.print(adc2_buf[i]); Serial.print("\t");
    //Serial.print(bin_buf[i]); Serial.print("\t");
  }

  Serial.println();
  Serial.print(temp);Serial.print("\t");
  }
