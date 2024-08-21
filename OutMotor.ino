void agv_running(){
  readProx();
  //DP[0]=1;
  //DP[1]=1;
  if (f_maju==1 && f_mundur==0)
  {
  //if(depan_obs==true)
  if(DP[2]==0)
  berhenti();
  else
  maju();
  }   
  else if (f_maju==0 && f_mundur==1)
  {
  if(DP[1]==0)
  berhenti();
  else
  mundur();
  }
  /*
      if (f_maju==1 && f_mundur==0){
          if(DP[2]==0||DP[3]==0){if (cnt_CNC>0)berhenti();}
          else {maju(); }      
      }
      
      else if (f_maju==0 && f_mundur==1){
          if(DP[0]==0||DP[1]==0) {if (cnt_CNC<14)berhenti();}
          else {mundur();}
      } 
  */

}

void maju(){

 maxspeed = maxspeed_maju;
    Sp_pwm = Sppwm_maju;
    Kp = Kp_maju;
    Kd = Kd_maju;
    Ki = Ki_maju;
/*
  if(DP[2]==0)
  {
    if(COUNT_LINE<=9)
    {
  
   
    }
    else
    {
    maxspeed = 120;
    Sp_pwm = 80;
    Kp = Kp_maju;
    Kd = Kd_maju;
    Ki = Ki_maju;
    }
  }
  else
  {
    if(COUNT_LINE<=9)
    {
  
    maxspeed = maxspeed_maju;
    Sp_pwm = Sppwm_maju;
    Kp = 5;
    Kd = 0.01;
    Ki = 0.001;
    }
    else
    {
    maxspeed = 120;
    Sp_pwm = 80;
    Kp = 5;
    Kd = 0.01;
    Ki = 0.001;
    }
  }
  
  */


  /*
  digitalWrite(SW_sensorD,HIGH);
  digitalWrite(SW_sensorB,LOW);
  */
 // Serial.println("");
  bacaSensor();
  PID_Mtr();
  Wrt_mtr(Pwm_kiri,Pwm_kanan);//////==================maju
  /*
  Serial.print(Error); 
   Serial.print("   ");
 //  Serial.print(cnt_CNC); 
 //  Serial.print("   ");
  Serial.print(Pwm_kiri);
   Serial.print("   ");
    Serial.println(Pwm_kanan);
    */
}

void mundur(){

  maxspeed = maxspeed_mundur;
    Sp_pwm = Sppwm_maju;
    Kp = Kp_mundur;
    Kd = Kd_mundur;
    Ki = Ki_mundur;
  
/*
  if(DP[2]==0)
  {
    if(COUNT_LINE<=18 && COUNT_LINE>9)
    {
    maxspeed = maxspeed_mundur;
    Sp_pwm = Sppwm_maju;
    Kp = Kp_mundur;
    Kd = Kd_mundur;
    Ki = Ki_mundur;
    }
    else if(COUNT_LINE>18 || COUNT_LINE==9)
    {
    maxspeed = 120;
    Sp_pwm = 80;
    Kp = Kp_mundur;
    Kd = Kd_mundur;
    Ki = Ki_mundur;
      
    }
  }
  else
  {
    if(COUNT_LINE<=18 && COUNT_LINE>9)
    {
    maxspeed = maxspeed_mundur;
    Sp_pwm = Sppwm_maju;
    Kp = 5;
    Kd = 0.01;
    Ki = 0.001;
    }
    else if(COUNT_LINE>18 || COUNT_LINE==9)
    {
    maxspeed = 120;
    Sp_pwm = 80;
    Kp = 5;
    Kd = 0.01;
    Ki = 0.001;
      
    }
    
  }
  */
 // Serial.println("mundur2");
  bacaSensor();
  PID_Mtr();
  Wrt_mtr(Pwm_kanan*(-1),Pwm_kiri*(-1));//////==================mundur
  /*
    Serial.print(Error); 
    Serial.print("   ");
//    Serial.print(cnt_CNC); 
//    Serial.print("   ");
    Serial.print(Pwm_kanan);
    Serial.print("   ");
    Serial.print(Pwm_kiri);
    Serial.println("   mundur");
    */
}

void berhenti(){
  Wrt_mtr(0,0);
}

void slowmaju(){
  for(int i=100; i>=10; i--)
  {
    Wrt_mtr(i,i);
    delay(5);
  }
}

void  Wrt_mtr(int kanan, int kiri){ 

   //motor kanan
      if (kiri>0){
         digitalWrite(L_MTRka,LOW);
         digitalWrite(R_MTRka,HIGH);
         analogWrite(PWM_MTRka,(kiri));  
      }
      else{
         digitalWrite(L_MTRka,HIGH);
         digitalWrite(R_MTRka,LOW);
         analogWrite(PWM_MTRka,(kiri*-1));  
      }
   //motor kiri
      if (kanan>0){
         digitalWrite(L_MTRki,HIGH);
         digitalWrite(R_MTRki,LOW);
         analogWrite(PWM_MTRki,kanan-13);  
      }
      else{
         digitalWrite(L_MTRki,LOW);
         digitalWrite(R_MTRki,HIGH);
         analogWrite(PWM_MTRki,(kanan*-1)+10);  
      }
}

void led_start()
{ for(int i=1;i<=8; i++)
  {
  digitalWrite(Lamp,HIGH);
  delay(500);
  digitalWrite(Lamp,LOW); 
  delay(500);
  }
}

void slowmundur(){
  for(int i=100; i>=10; i--)
  {
    Wrt_mtr(-i,-i);
    delay(5);
  }
}

void led_stop()
{
  digitalWrite(Lamp,LOW); 
}


void buzz_on(){
 digitalWrite(Buzzer,HIGH);
}



void buzz_on_2(){
 for(int i=1; i<=8; i++)
 {tone(Buzzer,6000);
 digitalWrite(Lamp,HIGH);
 delay(500); 
 tone(Buzzer,3000);
 delay(500);
 }
}
void buzz_off(){
   noTone(Buzzer);
  digitalWrite(Buzzer,LOW); 
}


void motor_crontrol(bool a, bool b)
{

   digitalWrite(SW_sensorD,a);
   digitalWrite(SW_sensorB,b);
}
