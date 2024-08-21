void PID_maju(){
    Sppwm_maju    = EEPROM.read(6);
    maxspeed_maju = EEPROM.read(7);
    Kp_maju       = EEPROM_readDouble(10);
    Kd_maju       = EEPROM_readDouble(15);
    Ki_maju       = EEPROM_readDouble(20);
}     
void PID_mundur(){
    Sppwm_mundur    = EEPROM.read(8);
    maxspeed_mundur = EEPROM.read(9);
    Kp_mundur       = EEPROM_readDouble(25);
    Kd_mundur       = EEPROM_readDouble(30);
    Ki_mundur       = EEPROM_readDouble(35);
  
}

void EEPROM_writeDouble(int ee, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
}

double EEPROM_readDouble(int ee)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return value;
}
