void PID_Mtr(){

  //Sp_sensor = 0;

  Error = Sp_sensor; 
  
  P     = Kp*Error;
  
  D1  = Kd*10;
  D2  = D1/Ts;
  D3  = Error-Last_error;
  D   = D2*D3;

  I1  = Ki/10;
  I2  = Error+Last_error;
  I3  = I1*I2;
  I   = I3*Ts;
  
  Last_error  = Error;
  Pd          = P+D;
  Pid         = Pd+I;


//Serial.println(D2);
  Pwm_kiri = Sp_pwm+Pid;
  Pwm_kanan = Sp_pwm-Pid;

  if(Pwm_kanan>maxspeed) Pwm_kanan=maxspeed;
  else if (Pwm_kanan<minspeed) Pwm_kanan=minspeed;
  
  if(Pwm_kiri>maxspeed)  Pwm_kiri=maxspeed;
  else if (Pwm_kiri<minspeed) Pwm_kiri=minspeed;
  
//  PID_prints = String(LastPV) + ";" + String(Pwm_kiri) + ";" + String(Pwm_kanan)+";" + String(cnt_CRP)+ "          ";
  
}
