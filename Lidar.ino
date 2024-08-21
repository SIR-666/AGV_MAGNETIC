void lidarr() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

    //perform data processing here...
    if (distance > 0 ) {
      //   Serial.print("Angle = ");



      // delay(1000);

   
        if((distance < 800)&& (quality!=0))
        { 
                        
          if (angle > 320 || angle < 40) {

            
              Serial.print(obs_left);
              Serial.print("  ");
              Serial.print(obs_right);
              Serial.print("  ");
              Serial.print(detect_obs);
               Serial.print(" | ");
              Serial.print(quality);
              Serial.print(" | ");
              Serial.print(angle);
              Serial.print(" | ");
              Serial.println(distance);
              
              
            detect_obs++;
              if(angle>320 && angle<390)
              {
                
                if(detect_obs==100 && detect_obs<=110)
                {
                  objectavoiding=true;
                obs_left=true;
                obs_right=false;
                depan_obs = false;
                count_detectagv=0;
                prevdetect = millis();
                }
                else if(detect_obs>5 && detect_obs<100)
                {
                depan_obs = true;
                }
              
              }
              else if(angle>5 && angle<40)
              {
                if(detect_obs==100 && detect_obs<=110)
                {
                  objectavoiding=true;
                obs_right=true;
                obs_left=false;
                depan_obs = false;
                count_detectagv=0;
                prevdetect = millis();
                }
                else if(detect_obs>5 && detect_obs<100)
                depan_obs = true;
              
              }
            if(angle>=390 && angle<=5)          
            depan_obs = true;
            /*
            Serial.print(angle);
            Serial.print(" | ");
            Serial.print(distance);
            Serial.print(" | ");
            Serial.println(quality);
            */
            filterSens++;
            if(filterSens>2)
            countiing=0;
            // Serial.println("DEPAANNNN objekkk");
          }
          
      }
      else if ((angle > 320 || angle < 40)&& (quality!=0)) {
        filterSens=0;
            //  Serial.print(" # ");
           //   Serial.println(countiing);
            
            countiing++;
            if (countiing>20)
            detect_obs=0;
            
            if (countiing>70) {
              depan_obs = false;
              if(angle>340 && angle<390)
              detect_obs=0;
            }
            if (countiing>70)
            {
               if(angle>0 && angle<40)
              detect_obs=0;
           // obs_right = false;
            //  obs_left = false;
              digitalWrite(Buzzer,LOW);
            }
            
          }
      
        //        else if (angle > 90 && angle < 150) {
        //          kanan_obs = true;
        //          Serial.println("KANANNNNN objekkk");
        //        }
        //        else if (angle > 30 && angle < 90) {
        //          kiri_obs = true;
        //          Serial.println("KIRIIIII objekkk");
        //        }

     
        

    }
  } else {
   // analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, 255);
      delay(1000);
    }
  }
}
void avoiding() {
  if (depan_obs == true ) {
    berhenti();
  }
}
