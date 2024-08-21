void bacaSensor(){

  if(sens_error<=1 && sens_error>=-1 && nsens!=0)
  Sp_sensor=0;
  else if(nsens!=0)
  {
    if(sens_error>0)
    {
      //Sp_sensor = sens_error-2;
      
        //Sp_sensor = ((2 * sens_error)-5);
        if(Sp_sensor<5)
        Sp_sensor = sens_error-1;
        else
        Sp_sensor = sens_error+2;
        
        
    }
    else
    {
      //Sp_sensor = sens_error+2;
     // Sp_sensor = ((2 * sens_error)+5);
     if(Sp_sensor>-5)
     Sp_sensor = sens_error+1;
     else
     Sp_sensor = sens_error-2;
     
    }
  }
  

  
  
  if(UNLOADING==true)
  {
    if(f_maju==1)
    {
      countunloading++;
      COUNT_LINE++;
     // Scan_LINE();
   // delay(300);
      Sp_sensor=0;

      if(NAV_COND==0)
      {
        if(COUNT_LINE<3)
        Stoploading();
        else if(COUNT_LINE==3)
        {
          String send_ser = COUNT_LINE+"/"+NAV_COND;
          leo.println(send_ser);
          EEPROM.write(10, f_maju);
          EEPROM.write(11, COUNT_LINE);
          EEPROM.write(12, NAV_COND);
          Stoploading();
          Reset();
          f_main=0;
        }
      }
      else if(NAV_COND==1)
      {
        if(COUNT_LINE>3 && COUNT_LINE<6)
        Stoploading();
        else if(COUNT_LINE==6)
        {
          String send_ser = COUNT_LINE+"/"+NAV_COND;
          leo.println(send_ser);
          EEPROM.write(10, f_maju);
          EEPROM.write(11, COUNT_LINE);
          EEPROM.write(12, NAV_COND);
          Stoploading();
          Reset();
          f_main=0;
        }
      }
      
    }
    if(f_mundur==1)
    {
      countunloading++;
      COUNT_LINE++;
      Serial.println("LOADING#LOADING");
      leo.println("LOADING#LOADING");
      String send_ser = COUNT_LINE+"/"+NAV_COND;
      leo.println(send_ser);
      
     if(NAV_COND==1)
      {
        if(COUNT_LINE==10)
        {
          slowmaju();
          berhenti();
          f_maju=0;
          f_mundur=1;
          delay(2000);
          obs_right = false;
          obs_left = false;
          CHANGE_DIRECT=false;
          leo.println("BACK#BACK");
          COUNT_LINE=3;  
          EEPROM.write(10, f_maju);
          EEPROM.write(11, COUNT_LINE);
          EEPROM.write(12, NAV_COND);
        }
      }
      
     // if(COUNT_LINE>2)
     // Stoploading();
     // if(COUNT_LINE==4)
     // Reset();
      
      
   // delay(300);
      Sp_sensor=0;
    
    }
    UNLOADING=false;
  }

  if(CHANGE_DIRECT==true)
  {
    if(f_maju==1)
    {
    slowmaju();
    berhenti();
    Reset();
    sing(2);
    sing(2);
    f_maju=0;
    f_mundur=1;
    f_main=0;
    obs_right = false;
      obs_left = false;
    CHANGE_DIRECT=false;
    leo.println("BACK#BACK");
    COUNT_LINE=6;
    
    if(NAV_COND==0)
    NAV_COND=1;
    else
    NAV_COND=0;
    
    String send_ser = COUNT_LINE+"/"+NAV_COND;
    leo.println(send_ser);
    f_main=0;
    EEPROM.write(10, f_maju);
    EEPROM.write(11, COUNT_LINE);
    EEPROM.write(12, NAV_COND);
  //  agv_running();
    }
    else if(f_mundur==1 && CHANGE_DIRECT==true)
    {
      obs_right = false;
      obs_left = false;
      slowmundur();
      berhenti();
     // Reset();
    //  sing(2);
    //  sing(2);
      //String PARKING="PARK#PARK";
      leo.println("START#START");
      delay(2000);
      f_maju=1;
      f_mundur=0;
      f_main=1;
      COUNT_LINE =0;
      CHANGE_DIRECT=false;
      COUNT_LINE=0;
      String send_ser = COUNT_LINE+"/"+NAV_COND;
      leo.println(send_ser);
      EEPROM.write(10, f_maju);
      EEPROM.write(11, COUNT_LINE);
      EEPROM.write(12, NAV_COND);
      //Reset();
    }
   
    
    
  }

  
  
  
}




void readProx(){
  DP[0] = digitalRead(Prox1);
  DP[1] = digitalRead(Prox2);
  DP[2] = digitalRead(Prox3);
  DP[3] = digitalRead(Prox4);
}
