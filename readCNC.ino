
void Stoploading()
{
  slowmaju();
  berhenti();
  sing(2);
  sing(2);
  buzz_off();
  led_stop(); 
  UNLOADING=false;
}

void Scan_LINE(){
      for(int i=0; i<10;i++)
      {
          if(LINE[i]==1 && COUNT_LINE==(i+1))
          {
            String LINENOW="LINE.LINE"+String(12-i)+"#";
            leo.println(LINENOW);
            Serial.print(COUNT_LINE);
            Serial.print(" | ");
            Serial.println(LINENOW);
            slowmaju();
            berhenti();
            sing(2);
            sing(2);
            sing(2);
            sing(2);
            buzz_off();
            led_stop(); 
            LINE[i]=0; 
            //agv_running();
            i=10;
            break;
            UNLOADING=false;
            
          }  
      }
      UNLOADING=false;
  
  //  if (CNC[1]==1 && cnt_CNC==1) {slowmaju();berhenti();sing(1);buzz_off();led_stop(); CNC[1]=0; jalan();Serial3.println("CNC7");Serial.println("CNC7");}
  //  if (CNC[2]==1 && cnt_CNC==2) {slowmaju();berhenti();sing(1);buzz_off();led_stop(); CNC[2]=0; jalan();Serial3.println("CNC6");Serial.println("CNC6");}
  //  if (CNC[3]==1 && cnt_CNC==3) {slowmaju();berhenti();sing(1);buzz_off();led_stop(); CNC[3]=0; jalan();Serial3.println("CNC5");Serial.println("CNC5");}
  //  if (CNC[4]==1 && cnt_CNC==4) {slowmaju();berhenti();sing(1);buzz_off();led_stop(); CNC[4]=0; jalan();Serial3.println("CNC4");Serial.println("CNC4");}
  //  if (CNC[5]==1 && cnt_CNC==5) {slowmaju();berhenti();sing(1);buzz_off();led_stop(); CNC[5]=0; jalan();Serial3.println("CNC3");Serial.println("CNC3");}
 //   if (CNC[6]==1 && cnt_CNC==6) {slowmaju();berhenti();sing(1);buzz_off();led_stop(); CNC[6]=0; jalan();Serial3.println("CNC2");Serial.println("CNC2");}
//    if (CNC[7]==1 && cnt_CNC==7) {slowmaju();berhenti();sing(1);buzz_off();led_stop(); CNC[7]=0; jalan();Serial3.println("CNC1 ");Serial.println("CNC1");}
    
}

void Reset(){
      COUNT_LINE =0;
//motor_crontrol(LOW,LOW);
    //  PID_maju();PID_mundur();
    obs_right = false;
      obs_left = false;
      berhenti();
      f_main=0;
   //   f_mundur=0;
      UNLOADING=false;
      CHANGE_DIRECT=false;
      //cnt_CNC=0; 
//      for(int i=1;i<8;i++){CNC[i]=0;}
}
