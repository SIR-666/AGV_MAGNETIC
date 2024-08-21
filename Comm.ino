void DataProcess(String data){
  //  data = data + "okke";
    String save_eeprom;
    String Code = splitValue(data, '#', 0);
    String Data = splitValue(data, '#', 1);
    
   // Serial.println(Code);
   // Serial.println(Data);

      if (Code=="BACK" || Code=="BACK#"){
      
      berhenti();
      f_maju=0;
      f_mundur=1;
      f_main=1;
      delay(2000);
      CHANGE_DIRECT=false;
      COUNT_LINE=9;
      }
      

      if (Code=="STRT" || Code=="STRT#"){
          Serial.println("jalan");
        //  analogWrite(RPLIDAR_MOTOR, 255);
          obs_right = false;
            obs_left = false;
          //CNC[0] = Data.substring(0,1).toInt();
          LINE[9] = Data.substring(0,1).toInt();
          LINE[8] = Data.substring(1,2).toInt();
          LINE[7] = Data.substring(2,3).toInt();
          LINE[6] = Data.substring(3,4).toInt();
          LINE[5] = Data.substring(4,5).toInt();
          LINE[4] = Data.substring(5,6).toInt();
          LINE[3] = Data.substring(6,7).toInt();
          LINE[2] = Data.substring(7,8).toInt();
          LINE[1] = Data.substring(8,9).toInt();
          LINE[0] = Data.substring(9,10).toInt();
          
          for(int i=0; i<10; i++)
          //Serial.println(LINE[i]);
          
          if(LINE[i]==1)
          {
          String statusLINE = "LINE"+String(10-i);
        //  Serial.println(statusLINE);
          }
          Serial.println("START#START");
          f_main=1;
         // f_mundur=0;
         // f_maju=1;
          CHANGE_DIRECT=false;
          count_detectagv=0;
          UNLOADING=false;
         
          //jalan();
      }

      else if (Code=="CONT" || Code=="CONT"){
          f_main=1;
      }
      
      else if (Code=="RST" || Code=="RST#"){
          Reset();
           f_mundur=0;
          f_maju=1;
          Serial.println("RESET");
          analogWrite(RPLIDAR_MOTOR, 0);
      }    

     

        
}

void SendData(String data){
 
      Serial.println(data); 
}

void leo_serial()
{       //  Serial.println("cek");
        while (leo.available()) {
        char inChar = (char)leo.read();             
        //Serial.print(inChar);
        if (inChar == '\n') {
            //datamasuk = inputString.substring(1);
            Serial.println(inputString);
            DataProcess(inputString);       
            inputString="";
         }
         else{
            inputString += inChar;
         }
    }  


}



String splitValue(String data, char separator, int index){
  int found = 0;
  int strIndex[]={0, -1};
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i) == separator || i==maxIndex){
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}
