#include <Adafruit_MCP3008.h>

Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = A3;
const unsigned int ADC_2_CS = A2;
const int reference_sig = 6;

void setup() 
{
  Serial.begin(115200);

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
}

void loop() 
{  
  int adc_buf[13];
  int bin_buf[13];  // binary array
  //cutoff values for binary conversion - will vary for each robot
  int cutoff[13] = {600,600,600,600,600,600,600,600,600,600,600,650,700}; 
  int sensor_sum = 0; // sum of sensor positions reading the line
  int sensor_count = 0; // count of the sensors 
  int center_pos;
  int error_sig;
  
  int t_start = micros();

  // Reading sensor array values
  for (int i = 0, j = 0; i < 13; i = i + 2) 
  {
    adc_buf[i] = adc1.readADC(j);
    if (i <12)
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
     center_pos = -1;
  }

  //caluclating the error signal
  error_sig = reference_sig - center_pos;

  int t_end = micros();

  for (int i = 0; i < 13; i++) 
  {
    Serial.print(bin_buf[i]); Serial.print("\t");
  }

  Serial.print(center_pos); Serial.print("\t");
  Serial.print(error_sig); Serial.print("\t");
  //Serial.print(t_end - t_start);
  Serial.println();

  delay(100);
}
