void processhex(uint32_t hexinput) {
  // put your main code here, to run repeatedly:
  uint32_t input =hexinput;
    
  uint32_t simpan = 0x0000;

  uint32_t hasil2 = 0x0000;
  uint32_t simpanahir = 0x0000;
  uint32_t input1 = input & 0x00ff;
  uint32_t input2 = input & 0xff00;
  uint32_t sensstop=input;
  hexnsens=0x0001;
  UNLOADING=false;
   
  for(int i=1; i<=16; i++)
  {
      sensstop = input & hexnsens;
      if(sensstop==0x0000)
      nsens++;
 
      hexnsens = hexnsens << 1;
  }

  // Serial.print(nsens);
 // Serial.print(" # ");
  

//  Serial.print(nsens);
//  Serial.print(" # ");
  //Serial.print(nsens);
 // Serial.println(" # ");
  if(nsens<=4)
  {
  
  simpan = input1;
  count_detectagv++;

  

  for(int i=0; i<8; i++)
  {
    uint32_t convert = simpan;
    uint32_t convert2 = input2;
    convert = convert & pengali3[i];
    convert2 = convert2 & pengali2[i];
  //Serial.print(hasil);
  //Serial.print(" # ");
//  Serial.print(convert,HEX);
    if(convert==0x0000)
    {
     hasil=hasil+(i+1); 
     pembagi++;
     }

     if(convert2==0x0000)
    {
     hasil2=hasil2+(i+1); 
     pembagi2++;
     }
  }

  float hasilahir=hasil;
  float hasilahir2=hasil2;
  
  if(hasil==0)
  {
  hasilahir=0;
  //pembagi=pembagi2;
  }
  else
  hasilahir = float(hasil)/pembagi;

  if(hasil2==0)
  {
   hasilahir2=0;
   //pembagi2=pembagi;
  }
  else
  hasilahir2 = float(hasil2)/pembagi2;
  
  
  

  float error_sens = hasilahir2-hasilahir;
  sens_error = error_sens; 

  /*
  Serial.print(input,HEX);
  Serial.print(" # ");
  Serial.print(hasil);
  Serial.print(" # ");
  Serial.print(hasilahir);
  Serial.print(" # ");
  Serial.print(hasil2);
  Serial.print(" # ");
  Serial.print(hasilahir2);
  Serial.print(" - ");
  
   
  Serial.print(error_sens);
  Serial.print("   ");
  Serial.print(Error);
  Serial.print("   ");
  Serial.print(Pwm_kiri);
  Serial.print("   ");
  Serial.println(Pwm_kanan);
  
 */
  pembagi=0;
  hasil=0;
  pembagi2=0;
  hasil2=0;
  count_stop=0;
  }
  else if(nsens<=8 && count_detectagv>5)
  {
    Serial.println("unloading");
   
    UNLOADING=true;
    //delay(500);
  //  nsens=0;
   // agv_running();
   // UNLOADING=false;
    count_detectagv=0;
    count_stop=0;
  }
  else if(nsens>8 && count_detectagv>2)
  {
    Serial.println("stop");
    CHANGE_DIRECT=true;
    count_detectagv=0;
    //agv_running();
  //  nsens=0;
    count_stop=0;
  }
  else if (nsens==0)
  count_detectagv=0;
  /*
  else
  {
    
    count_detectagv++;
  }
  */
  
   if (f_main==1){
       //Serial.print("jalan");
        agv_running();
    }
    else{
        berhenti();
    }
  
    
  nsens=0; 
 // delay(100);
    
}

void SerialEven_AGV1()
{   
       if (Serial1.available()>0)  //Look for data from other Arduino
       {
            
        count_i++;
          
          byteReceived = Serial1.read();    // Read received byte
        
         // Serial.print(byteReceived,HEX);
         // Serial.print(" | ");
         // Serial.println(count_i);
          if(count_i==1)
          {
            if(byteReceived!=0x01)
            count_i=0;
          }
          else if(count_i==2)
          {
            if(byteReceived!=0x03)
            count_i=0;
          }
          else if(count_i==3)
          {
            if(byteReceived!=0x04)
            count_i=0;
          }

       
          if(count_i==6)
          {
         // Serial.print(byteReceived,HEX);
          inputser=byteReceived;
          inputser = inputser << 8;
          
           
          }

          if(count_i==7)
          {
            
          inputser=inputser | byteReceived;
        //  Serial.println(input,HEX);
         // if(f_mundur==1)
          processhex(inputser);
           
          }

          cek =byteReceived;
 
       if(count_i>8)
       count_i=0;

        
         
      
       }  
       
}

void SerialEven_AGV2()
{   
       if (Serial2.available()>0)  //Look for data from other Arduino
       {
            
        count_i_agv2++;
          
          byteReceived_agv2 = Serial2.read();    // Read received byte
        
         // Serial.print(byteReceived,HEX);
         // Serial.print(" | ");
         // Serial.println(count_i);
          if(count_i_agv2==1)
          {
            if(byteReceived_agv2!=0x01)
            count_i_agv2=0;
          }
          else if(count_i_agv2==2)
          {
            if(byteReceived_agv2!=0x03)
            count_i_agv2=0;
          }
          else if(count_i_agv2==3)
          {
            if(byteReceived_agv2!=0x04)
            count_i_agv2=0;
          }

       
          if(count_i_agv2==6)
          {
         // Serial.print(byteReceived,HEX);
          inputser_agv2=byteReceived_agv2;
          inputser_agv2 = inputser_agv2 << 8;
          
           
          }

          if(count_i_agv2==7)
          {
            
          inputser_agv2=inputser_agv2 | byteReceived_agv2;
        //  Serial.println(input,HEX);
         // if(f_maju==1)
          processhex(inputser_agv2);
           
          }

          cek_agv2 =byteReceived_agv2;
 
       if(count_i_agv2>8)
       count_i_agv2=0;

        
         
      
       }  
       
}
