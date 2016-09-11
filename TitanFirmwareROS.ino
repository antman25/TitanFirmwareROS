

#include "common.h"


void SendMotorPacket(char addr, char cmd, char data) 
{ 
  SabertoothSerial.write(addr);   
  SabertoothSerial.write(cmd);   
  SabertoothSerial.write(data); 
  SabertoothSerial.write((addr + cmd + data) & 127); 
} 


void InitializeIMU()
{
  if(!bno.begin())
  {

  }

  delay(1000);
 
  bno.setExtCrystalUse(true);
}

void setLeftMotorPower(float power)
{
  if (power > 100.0)
    power = 100.0;
    
  if (power < -100.0)
    power = -100.0;
    
  char val = map(abs(power), 0, 100, 0, 127); 
  if (power >= 0.0)
  {
    SendMotorPacket(SABERTOOTH_ADDR, SABERTOOTH_CMD_MOTOR2_FWD, val);
  }
  else
  {
    SendMotorPacket(SABERTOOTH_ADDR, SABERTOOTH_CMD_MOTOR2_REV, val);
  }
}


void setRightMotorPower(float power)
{
  if (power > 100.0)
    power = 100.0;
    
  if (power < -100.0)
    power = -100.0;
    
  char val = map(abs(power), 0, 100, 0, 127); 
  if (power >= 0.0)
  {
    SendMotorPacket(SABERTOOTH_ADDR, SABERTOOTH_CMD_MOTOR1_FWD, val);
  }
  else
  {
    SendMotorPacket(SABERTOOTH_ADDR, SABERTOOTH_CMD_MOTOR1_REV, val);
  }
}


void publishDebug()
{
  char output[80];
  
  //sprintf(output,"%i-%i-%i  %i:%i:%i ** %f - %f",GPS.month, GPS.day, GPS.year, GPS.hour, GPS.minute, GPS.seconds,(float)setpointRightVel,(float)motorRightOutput);
  
  //sprintf(output, "Heading: %f - %f - %f\nLeft Setpoint: %f\nLeft Output: %f\nLeftVel: %f\nRight Setpoint: %f\nRight Output: %f\nRightVel: %f\n",  (float)currentHeading, sin(currentHeading / 2.0), cos(currentHeading / 2.0),(float)setpointLeftVel,(float)motorLeftOutput, (float)currentLeftVel, (float)setpointRightVel,(float)motorRightOutput,(float)currentRightVel);
  sprintf(output,"Left: %f  -- Right: %f\n",motorLeftOutput,motorRightOutput);
  debug.data = output;
  pubDebug.publish( &debug );
}

void updateMotors()
{
  setLeftMotorPower(motorLeftOutput);
  setRightMotorPower(motorRightOutput);
}

void LeftMotorCmd_cb( const std_msgs::Float32 &msg)
{
  motorLeftOutput = msg.data;
  updateMotors();
  timerMotorTimeout = millis();
}
void RightMotorCmd_cb( const std_msgs::Float32 &msg)
{
  motorRightOutput = msg.data;
  updateMotors();
  timerMotorTimeout = millis();
}

void checkTimers()
{
  if (millis() - timerMotorTimeout > TIMEOUT_MOTOR_CMD)
  {
    motorLeftOutput = 0.0f;
    motorRightOutput = 0.0f;
  }

  if (millis() - timerMotorUpdate > UPDATE_RATE_MOTOR)
  {
    timerMotorUpdate = millis();
    updateMotors();
  }

  if (millis() - timerEncoder > UPDATE_RATE_ENCODER)
  {
    timerEncoder = millis();
    updateEncoders();
    
  }
  
  if (millis() - timerIMU > UPDATE_RATE_IMU)
  {
   timerIMU = millis();
   
   publishIMU();
  }

  if (millis() - timerDebug > UPDATE_RATE_DEBUG)
  {
   timerDebug = millis();
   publishDebug();
  }
  
}

ros::Subscriber<std_msgs::Float32> subLeftMotorCmd("lmotor_cmd", LeftMotorCmd_cb);
ros::Subscriber<std_msgs::Float32> subRightMotorCmd("rmotor_cmd", RightMotorCmd_cb);



void setup() {

  Wire.begin(); // join i2c bus (address optional for master)
  delayMicroseconds(10000); //wait for motor driver to initialization
  SabertoothSerial.begin(9600);
  delay(2000);
  SendMotorPacket(SABERTOOTH_ADDR,SABERTOOTH_CMD_MIN_VOLTAGE, MIN_VOLTAGE_VAL);
  SendMotorPacket(SABERTOOTH_ADDR,SABERTOOTH_CMD_TIMEOUT, CMD_TIMEOUT_VAL);

  
 
  /*setLeftMotorPower(50);
  setRightMotorPower(50);
  delay(1000);*/
  
  //InitializePID();

  nh.initNode();
  //broadcaster.init(nh);

  nh.advertise(pubLeftEncoder);
  nh.advertise(pubRightEncoder);
  nh.advertise(pubIMU);
  nh.advertise(pubDebug);
  
  
  nh.subscribe(subLeftMotorCmd);
  nh.subscribe(subRightMotorCmd);
  
  InitializeIMU();
  rosInitialized = true;
}


void publishIMU()
{
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE );
  imu::Quaternion quat = bno.getQuat();
  //int8_t temp = bno.getTemp();

  imuMsg.linear_acceleration.x = accel.x();
  imuMsg.linear_acceleration.y = accel.y();
  imuMsg.linear_acceleration.z = accel.z();
  //imuMsg.linear_acceleration_covariance = { 0.04 , 0 , 0, 0 , 0.04, 0, 0 , 0 , 0.04 };
  imuMsg.linear_acceleration_covariance[0] = 0.04;
  imuMsg.linear_acceleration_covariance[4] = 0.04;
  imuMsg.linear_acceleration_covariance[8] = 0.04;

 
  imuMsg.angular_velocity.x = gyro.x();
  imuMsg.angular_velocity.y = gyro.y();
  imuMsg.angular_velocity.z = gyro.z();
  //imuMsg.angular_velocity_covariance = { 0.02, 0 , 0, 0 , 0.02, 0, 0 , 0 , 0.02 };
  imuMsg.angular_velocity_covariance[0] = 0.02;
  imuMsg.angular_velocity_covariance[4] = 0.02;
  imuMsg.angular_velocity_covariance[8] = 0.02;

  imuMsg.orientation.x = quat.x();
  imuMsg.orientation.y = quat.y();
  imuMsg.orientation.z = -quat.z();
  imuMsg.orientation.w = quat.w();
  //imuMsg.orientation_covariance = { 0.0025 , 0 , 0, 0, 0.0025, 0, 0, 0, 0.0025 };
  imuMsg.orientation_covariance[0] = 0.0025;
  imuMsg.orientation_covariance[4] = 0.0025;
  imuMsg.orientation_covariance[8] = 0.0025;

  char id[] = "imu_link";
  imuMsg.header.frame_id = id;
  imuMsg.header.stamp=nh.now();
  imuMsg.header.seq = seq;
  seq = seq + 1;
  pubIMU.publish(&imuMsg);
}


void updateEncoders()
{
  long countLeft = -encoderLF.read();
  long countRight = encoderRF.read();
  
  encoderLeftVal.data = countLeft;
  pubLeftEncoder.publish(&encoderLeftVal);

  encoderRightVal.data = countRight;
  pubRightEncoder.publish(&encoderRightVal);
  

  
  
 
}

void loop() {
  if (rosInitialized == true)
    checkTimers();

  nh.spinOnce();
  delay(1);
}
