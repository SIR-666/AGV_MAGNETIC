void IMU_read()
{
  readAcceleration();
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
  
  // === Read gyroscope (on the MPU6050) data === //
  previousTime_ = currentTime_;
  currentTime_ = micros();
  elapsedTime = (currentTime_ - previousTime_) / 1000000; // Divide by 1000 to get seconds
  readGyro();
  // Correct the outputs with the calculated error values
  GyroX -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
  //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  angle = roll; //if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three  
}

void readAcceleration() {
  accel.x = icg.getAccelDataX();
  AccX = accel.x; 
  accel.y = icg.getAccelDataY();
  AccY = accel.y;
  accel.z = icg.getAccelDataZ();
  AccZ = accel.z;
}

void readGyro() {
  gyro.x = icg.getGyroDataX();
  GyroX = gyro.x; 
  gyro.y = icg.getGyroDataY();
  GyroY = gyro.y;
  gyro.z = icg.getGyroDataZ();
  GyroZ = gyro.z;
}

void turn_left()
{
  Wrt_mtr(100,50);
}

void forward()
{
  Wrt_mtr(100,113);
}

void turn_right()
{
  Wrt_mtr(50,100);
}
