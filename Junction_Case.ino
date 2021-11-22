void junction(intersection){
  intersection = 0;
  for (int i = 0; i<13; i++){
    if(adc_buf[i] < cutoff[i]){
      if(i<6){
        left_sensor_count++;
      }
      if (i>6){
        right_sensor_count++;
      }   
    }
  }
    if (left_sensor_count > 3){
      left = 1;
    }
    if (right_sensor_count > 3){
      right = 1;
    }
      if(right == 1 && left ==0){
        moveForward();
        delay(100);
        moveStop();
      if(adc_buf[6] > cutoff[6]){
        Serial.println(" right turn ");
        intersection = 1;
      }
      if(adc_buf[6] > cutoff[6]{
        Serial.println("right turn Junction ( |-)");
        intersection = 2;
      }
    }
    if(right == 0 && left == 1){
      moveForward();
      delay(100);
      moveStop();
      if(adc_buf[6] > cutoff[6]){
        Serial.println(" Left Turn ");
        intersection = 3;
      }
      if(adc_buf[6] < curoff[6]){
        Serial.println(" Left turn junction ( -|)");
        intersection = 4;
      }
    }
    if( right == 1 && left == 1){
      moveForward();
      delay(100);
      moveStop();
      if(adc_buf[6] > cutoff[6]{
        Serial.println(" T Junction ");
        intersection = 5;
      }
      if(adc_buf[6] < cutoff[6]){
        Serial.println( " + Junction ");
        intersection = 6;
      }
    }
  
}


void LeftRight(left , right){
   for (int i = 0; i<13; i++){
             if(adc_buf[i] < cutoff[i]){
               if(i<6){
                left_sensor_count++;
               }
               if (i>6){
                right_sensor_count++;
               }
             }
    }
    left = left_sensor_count;
    right = right_sensor_count;
}
