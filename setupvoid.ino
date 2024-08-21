void setupvoid()
{
  while(!Serial){                                                     //Waiting for USB Serial COM port to open.
  }
  
  Serial.print("Initialization sensor...");
  /**
   * @fn begin
   * @brief Initialize the sensor, after initialization, all sensors are turned off, and the corresponding configuration
   * @n needs to be turned on through enableSensor.
   * @param mode: Enum variable,from eDataReadMode_t, configure to read sensor data from FIFO or register?
   * @n     eRegMode: Read sensor data from registers.
   * @n     eFIFOMode:Read sensor data from 512 bytes FIFO.
   * @note  Read from FIFO, accelerometer, gyroscope and temperature must all be enabled, 
   * @n and the internal sampling rate must be configured to be consistent. 
   * @return status:
   * @n      0 :   Initialization success.
   * @n      -1:   Interface initialization failed(IIC or SPI).
   * @n      -2:   Failed to read the device ID, the ID is not 0x91
   */
  while(icg.begin(/*mode=*/icg.eRegMode) != 0){
      Serial.println("failed. Please check whether the hardware connection is wrong.");
      delay(1000);
      Serial.print("Initialization sensor...");
  }
  Serial.println("done.");
  
  Serial.print("ICG20660L Device ID: 0x");
  Serial.println(icg.readID(), HEX);
  
  /**
   * @fn enableSensor
   * @brief Enable sensor, including Accel of xyz axis, Gyro of xyz, temperature. 
   * @param bit: 8-bit byte data. Each bit represents enabling a function bit, as shown in the following table:
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n |       bit7      |     bit6     |      bit5   |    bit4     |     bit3    |     bit2   |    bit1    |    bit0    |
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n |     reserve     |    reserve   | eAccelAxisX | eAccelAxisY | eAccelAxisZ | eGyroAxisX | eGyroAxisY | eGyroAxisZ |
   * @n |                                |            eAccelAxisXYZ                |           eGyroAxisXYZ               |
   * @n |                                |                                eAxisAll                                        |
   * @n -------------------------------------------------------------------------------------------------------------------
   * @n   bit0:  Z-axis of gyro and temperature.
   * @n   bit1:  Y-axis of gyro and temperature.
   * @n   bit2:  X-axis of gyro and temperature.
   * @n   bit3:  Z-axis of acceleration.
   * @n   bit4:  Z-axis of acceleration.
   * @n   bit5:  Z-axis of acceleration.
   * @n   bit6:  reserve.
   * @n   bit7:  reserve.
   * @note Enabling any axis of the gyroscope will automatically enable the on-board temperature sensor.
   * @n   eGyroAxisZ: The bit0 of the bit, enable gyro's z axis and temperature.
   * @n   eGyroAxisY: The bit1 of the bit, enable gyro's y axis and temperature.
   * @n   eGyroAxisX: The bit2 of the bit, enable gyro's X axis and temperature.
   * @n   eAccelAxisZ: The bit3 of the bit, enable accel's z axis.
   * @n   eAccelAxisY: The bit4 of the bit, enable Accel's y axis.
   * @n   eAccelAxisX: The bit5 of the bit, enable Accel's X axis.
   * @n   eGyroAxisXYZ or eGyroAxisX|eGyroAxisY|eGyroAxisZ: The bit0/bit1/bit2 of the bit, enable gyro's xyz axis and temperature.
   * @n   eAccelAxisXYZ or eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit3/bit4/bit5 of the bit, enable Accel's xyz axis.
   * @n   eAxisAll or eGyroAxisX|eGyroAxisY|eGyroAxisZ|eAccelAxisX|eAccelAxisY|eAccelAxisZ: The bit0/bit1/bit2/bit3/bit4/bit5 of the bit,
   * @n enable temperature, Accel's and gyro's xyz axis. 
   */
  icg.enableSensor(icg.eGyroAxisXYZ);
  icg.enableSensor(icg.eAccelAxisXYZ);
  //icg.enableSensor(icg.eGyroAxisX|icg.eGyroAxisY|icg.eGyroAxisZ);
  /**
   * @fn configGyro
   * @brief Config of gyro's full scale, dlpf bandwidth and internal sample rate. 
   * @param scale  The full scale of gyro, unit: dps(Degrees per second).
   * @n     eFSR_G_125DPS:  The full scale range is ±125 dps.
   * @n     eFSR_G_250DPS:  The full scale range is ±250 dps.
   * @n     eFSR_G_500DPS:  The full scale range is ±500 dps.
   * @param bd  Set 3-db bandwidth.
   * @n     eGyro_DLPF_8173_32KHZ: When the signal is equal to or greater than 8173Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
   * @n     eGyro_DLPF_3281_32KHZ: When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 32KHz.
   * @n     eGyro_DLPF_250_8KHZ:   When the signal is equal to or greater than 250Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
   * @n     eGyro_DLPF_176_1KHZ:   When the signal is equal to or greater than 176Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eGyro_DLPF_92_1KHZ:    When the signal is equal to or greater than 92Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 1KHz.
   * @n     eGyro_DLPF_3281_8KHZ:  When the signal is equal to or greater than 3281Hz, there will be obvious attenuation, 3-db attenuation, and the internal sampling rate is 8KHz.
   * @note When the gyroscope and accelerometer are both enabled, if the sensor data is read through the FIFO,
   * @n the internal sampling rate of the gyroscope and accelerometer must be the same.
   */
  icg.configGyro(icg.eFSR_G_250DPS, icg.eGyro_DLPF_8173_32KHZ);
  icg.configAccel(icg.eFSR_A_16G, icg.eAccel_DLPF_99_1KHZ);
  /**
   * @fn setSampleDiv
   * @brief Set sample rate divider. 
   * @param div  Sample rate divider, the range is 0~255.
   * @n     Sampling rate = internal sampling rate/(div+1)
   * @note  If the accelerometer configuration is in low power consumption mode, that is, the formal parameter lowPowerFlag of the configAccel function is true, 
   * @n the sampling rate must match the output rate of the formal parameter odr of configAccel, as shown in the following table:
   * @n ----------------------------------------------------------------------------
   * @n |                           configAccel                    |  setSampleDiv  |
   * @n ----------------------------------------------------------------------------|
   * @n |            bd             |      odr      | lowPowerFlag |      div       |
   * @n ----------------------------------------------------------------------------|
   * @n |            X              |       X       |    false     |      0~255     |
   * @n ----------------------------------------------------------------------------|
   * @n |                           |  eODR_125Hz   |    true      |        7       |
   * @n |                           |-----------------------------------------------|
   * @n |bd of supporting low power |  eODR_250Hz   |    true      |        3       |
   * @n |consumption mode           |-----------------------------------------------|
   * @n |                           |  eODR_500Hz   |    true      |        1       |
   * @n |---------------------------------------------------------------------------|
   */
  icg.setSampleDiv(19);

  sIcg20660SensorData_t  gyro;
  sIcg20660SensorData_t  accel;
  accel.x = icg.getAccelDataX();
  accel.y = icg.getAccelDataY();
  accel.z = icg.getAccelDataZ();
  gyro.x = icg.getGyroDataX();
  gyro.y = icg.getGyroDataY();
  gyro.z = icg.getGyroDataZ();  
}

void calculateError() {
  //When this function is called, ensure the car is stationary. See Step 2 for more info


  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  
  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += icg.getGyroDataX();
    GyroErrorY += icg.getGyroDataY();
    GyroErrorZ += icg.getGyroDataZ();
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}
