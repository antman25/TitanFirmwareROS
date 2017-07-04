#include "common.h"



void InitializeIMU()
{
  if(!bno.begin())
  {
  
  }

  delay(1000);
 
  bno.setExtCrystalUse(true);
}

int getAngleTime1(double angle)
{
  return map(angle, -180, 180, 900, 2350);     // scale it to use it with the servo (value between 0 and 180) 
}

int getAngleTime2(double angle)
{
  return map(angle, -180, 180, 850, 2250);     // scale it to use it with the servo (value between 0 and 180) 
}

int getAngleTime3(double angle)
{
  return map(angle, -90, 90, 850, 2200);     // scale it to use it with the servo (value between 0 and 180) 
}

/*void InitializeGPS()
{
  GPS.begin(115200);
  //GPS.sendCommand(PMTK_SET_BAUD_115200);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
}*/


void publishDebug()
{
  //char output[80];
  
  //sprintf(output,"%i-%i-%i  %i:%i:%i ** %f - %f",GPS.month, GPS.day, GPS.year, GPS.hour, GPS.minute, GPS.seconds,(float)GPS.lat,(float)GPS.lon);
  
  //sprintf(output, "Heading: %f - %f - %f\nLeft Setpoint: %f\nLeft Output: %f\nLeftVel: %f\nRight Setpoint: %f\nRight Output: %f\nRightVel: %f\n",  (float)currentHeading, sin(currentHeading / 2.0), cos(currentHeading / 2.0),(float)setpointLeftVel,(float)motorLeftOutput, (float)currentLeftVel, (float)setpointRightVel,(float)motorRightOutput,(float)currentRightVel);
  //sprintf(output,"Left: %f  -- Right: %f\n",motorLeftOutput,motorRightOutput);
  //sprintf(output,"%i - %i - %i - %i\n", spFrontLeftMotor, spFrontRightMotor, spRearLeftMotor, spRearRightMotor);
  //debug.data = output;
  //pubDebug.publish( &debug );
}

void setFrontLeftSpeed(int speed)
{
  roboclaw.SpeedM1(ROBOCLAW_FRONT_ID,speed);
}

void setFrontRightSpeed(int speed)
{
  roboclaw.SpeedM2(ROBOCLAW_FRONT_ID,speed);
}

void setRearLeftSpeed(int speed)
{
  roboclaw.SpeedM1(ROBOCLAW_REAR_ID,speed);
}

void setRearRightSpeed(int speed)
{
  roboclaw.SpeedM2(ROBOCLAW_REAR_ID,speed);
}


void updateMotors()
{
  setFrontLeftSpeed(spFrontLeftMotor);
  setFrontRightSpeed(spFrontRightMotor);
  setRearLeftSpeed(spRearLeftMotor);
  setRearRightSpeed(spRearRightMotor);
}

void cbFrontLeftMotorCmd( const std_msgs::Int64 &msg)
{
  spFrontLeftMotor = (int)msg.data;
  timerMotorTimeout = millis();
}
void cbFrontRightMotorCmd( const std_msgs::Int64 &msg)
{
  spFrontRightMotor = (int)msg.data;
  timerMotorTimeout = millis();
}

void cbRearLeftMotorCmd( const std_msgs::Int64 &msg)
{
  spRearLeftMotor = (int)msg.data;
  timerMotorTimeout = millis();
}
void cbRearRightMotorCmd( const std_msgs::Int64 &msg)
{
  spRearRightMotor = (int)msg.data;
  timerMotorTimeout = millis();
}

void cbServo1Cmd( const std_msgs::Float32 &msg)
{
  spServo1 = (float)msg.data;
  servo1.write(getAngleTime1(spServo1)); 
}

void cbServo2Cmd( const std_msgs::Float32 &msg)
{
  spServo2 = (float)msg.data;
  servo2.write(getAngleTime2(spServo2));
}

void cbServo3Cmd( const std_msgs::Float32 &msg)
{
  spServo3 = (float)msg.data;
  servo3.write(getAngleTime3(spServo3));
}

void checkTimers()
{
  if (millis() - timerMotorTimeout > TIMEOUT_MOTOR_CMD)
  {
    spFrontLeftMotor = 0;
    spFrontRightMotor = 0;
    spRearLeftMotor = 0;
    spRearRightMotor = 0;  
  }

  if (millis() - timerMotorInfo > UPDATE_RATE_MOTOR_INFO)
  {
    timerMotorInfo = millis();
    updateMotorInfo();
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
   
   //publishIMU();
  }

  if (millis() - timerDebug > UPDATE_RATE_DEBUG)
  {
   timerDebug = millis();
   publishDebug();
  }
  
}


ros::Subscriber<std_msgs::Int64> subFrontLeftMotorCmd("vel_sp_fl_wheel", cbFrontLeftMotorCmd);
ros::Subscriber<std_msgs::Int64> subFrontRightMotorCmd("vel_sp_fr_wheel", cbFrontRightMotorCmd);
ros::Subscriber<std_msgs::Int64> subRearLeftMotorCmd("vel_sp_rl_wheel", cbRearLeftMotorCmd);
ros::Subscriber<std_msgs::Int64> subRearRightMotorCmd("vel_sp_rr_wheel", cbRearRightMotorCmd);

ros::Subscriber<std_msgs::Float32> subServo1Cmd("servo1_angle_cmd", cbServo1Cmd);
ros::Subscriber<std_msgs::Float32> subServo2Cmd("servo2_angle_cmd", cbServo2Cmd);
ros::Subscriber<std_msgs::Float32> subServo3Cmd("servo3_angle_cmd", cbServo3Cmd);

void setup() {
  
  servo1.attach(5,850,2350);  // attaches the servo on pin 9 to the servo object 
  servo2.attach(6,850,2350);  // attaches the servo on pin 9 to the servo object 
  servo3.attach(4,850,2350);  // attaches the servo on pin 9 to the servo object 

  servo1.write(getAngleTime1(0)); 
  servo2.write(getAngleTime2(0));
  servo3.write(getAngleTime3(0));
  
  
  roboclaw.begin(57600);
  
  nh.initNode();
  //nh_bluetooth.initNode();

  //nh_bluetooth.advertise(chatter);
  
  nh.advertise(pubFrontLeftEncoder);
  nh.advertise(pubFrontRightEncoder);
  nh.advertise(pubRearLeftEncoder);
  nh.advertise(pubRearRightEncoder);
  
  nh.advertise(pubIMURaw);
  nh.advertise(pubIMUFiltered);
  nh.advertise(pubMag);
  nh.advertise(pubTemp);
  nh.advertise(pubDebug);

 

  nh.advertise(pubVoltageFrontMain);
  nh.advertise(pubVoltageRearMain);
  nh.advertise(pubVoltageFrontLogic);
  nh.advertise(pubVoltageRearLogic);
  nh.advertise(pubCurrentFrontLeft);
  nh.advertise(pubCurrentFrontRight);
  nh.advertise(pubCurrentRearLeft);
  nh.advertise(pubCurrentRearRight);
  
  
  nh.subscribe(subFrontLeftMotorCmd);
  nh.subscribe(subFrontRightMotorCmd);
  nh.subscribe(subRearLeftMotorCmd);
  nh.subscribe(subRearRightMotorCmd);

  nh.subscribe(subServo1Cmd);
  nh.subscribe(subServo2Cmd);
  nh.subscribe(subServo3Cmd);
  
  //InitializeIMU();
  //InitializeGPS();
  rosInitialized = true;
}


void publishIMU()
{
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE );
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  int8_t temp = bno.getTemp();


  char id[] = "imu_link";
  imuRawMsg.header.frame_id = id;
  imuRawMsg.header.stamp=nh.now();
  imuRawMsg.header.seq = seq;
  seq = seq + 1;

  imuRawMsg.linear_acceleration.x = accel.x();
  imuRawMsg.linear_acceleration.y = accel.y();
  imuRawMsg.linear_acceleration.z = accel.z();
  imuRawMsg.linear_acceleration_covariance[0] = -1;
 
  imuRawMsg.angular_velocity.x = gyro.x();
  imuRawMsg.angular_velocity.y = gyro.y();
  imuRawMsg.angular_velocity.z = gyro.z();
  imuRawMsg.angular_velocity_covariance[0] = -1;
  
  pubIMURaw.publish(&imuRawMsg);

  
  
  imuFilteredMsg.header.frame_id = id;
  imuFilteredMsg.header.stamp=nh.now();
  imuFilteredMsg.header.seq = seq;
  seq = seq + 1;
  
  imuFilteredMsg.orientation.x = quat.x();
  imuFilteredMsg.orientation.y = quat.y();
  imuFilteredMsg.orientation.z = quat.z();
  imuFilteredMsg.orientation.w = quat.w();  
  imuFilteredMsg.orientation_covariance[0] = -1;

  imuFilteredMsg.linear_acceleration.x = linear_accel.x();
  imuFilteredMsg.linear_acceleration.y = linear_accel.y();
  imuFilteredMsg.linear_acceleration.z = linear_accel.z();
  imuFilteredMsg.linear_acceleration_covariance[0] = -1;
  
  imuFilteredMsg.angular_velocity.x = gyro.x();
  imuFilteredMsg.angular_velocity.y = gyro.y();
  imuFilteredMsg.angular_velocity.z = gyro.z();
  imuFilteredMsg.angular_velocity_covariance[0] = -1;
 
  pubIMUFiltered.publish(&imuFilteredMsg);


  magMsg.header.frame_id = id;
  magMsg.header.stamp=nh.now();
  magMsg.header.seq = seq;
  seq = seq + 1;
  magMsg.magnetic_field.x = mag.x() / 1000000.0;
  magMsg.magnetic_field.y = mag.y() / 1000000.0;
  magMsg.magnetic_field.z = mag.z()/ 1000000.0;

  pubMag.publish(&magMsg);


  tempMsg.header.frame_id = id;
  tempMsg.header.stamp=nh.now();
  tempMsg.header.seq = seq;
  seq = seq + 1;
  tempMsg.temperature = temp;
  
  pubTemp.publish(&tempMsg);
  
}


void updateEncoders()
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;  
  int32_t encFrontLeft = roboclaw.ReadEncM1(ROBOCLAW_FRONT_ID, &status1, &valid1);
  int32_t encFrontRight = roboclaw.ReadEncM2(ROBOCLAW_FRONT_ID, &status2, &valid2);
  int32_t encRearLeft = roboclaw.ReadEncM1(ROBOCLAW_REAR_ID, &status3, &valid3);
  int32_t encRearRight = roboclaw.ReadEncM2(ROBOCLAW_REAR_ID, &status4, &valid4);

  encoderFrontLeftVal.data = encFrontLeft;
  pubFrontLeftEncoder.publish(&encoderFrontLeftVal);

  encoderFrontRightVal.data = encFrontRight;
  pubFrontRightEncoder.publish(&encoderFrontRightVal);

  encoderRearLeftVal.data = encRearLeft;
  pubRearLeftEncoder.publish(&encoderRearLeftVal);

  encoderRearRightVal.data = encRearRight;
  pubRearRightEncoder.publish(&encoderRearRightVal); 
}

void updateMotorInfo()
{
  bool valid1,valid2,valid3,valid4;  
  int16_t currentFrontLeft = 0;
  int16_t currentFrontRight = 0;
  int16_t currentRearLeft = 0;
  int16_t currentRearRight = 0;
  
  uint16_t batteryFrontMainVoltage = roboclaw.ReadMainBatteryVoltage(ROBOCLAW_FRONT_ID, &valid1);
  uint16_t batteryRearMainVoltage = roboclaw.ReadMainBatteryVoltage(ROBOCLAW_REAR_ID, &valid2);
  
  uint16_t batteryFrontLogicVoltage = roboclaw.ReadLogicBatteryVoltage(ROBOCLAW_FRONT_ID, &valid3);
  uint16_t batteryRearLogicVoltage = roboclaw.ReadLogicBatteryVoltage(ROBOCLAW_REAR_ID, &valid4);

  bool ret1 =  roboclaw.ReadCurrents(ROBOCLAW_FRONT_ID, currentFrontRight, currentFrontLeft);
  bool ret2 =  roboclaw.ReadCurrents(ROBOCLAW_REAR_ID, currentRearRight, currentRearLeft);

  if (ret1 == false)
  {
    currentFrontRight = -1.0;
    currentFrontLeft = -1.0;
  }

  if (ret2 == false)
  {
    currentRearRight = -1.0;  
    currentRearLeft = -1.0;
  }
  

  voltageFrontMainVal.data = (float)batteryFrontMainVoltage / 10.0f;
  pubVoltageFrontMain.publish(&voltageFrontMainVal);

  voltageRearMainVal.data = (float)batteryRearMainVoltage / 10.0f;
  pubVoltageRearMain.publish(&voltageRearMainVal);

  voltageFrontLogicVal.data = (float)batteryFrontLogicVoltage / 10.0f;
  pubVoltageFrontLogic.publish(&voltageFrontLogicVal);

  voltageRearLogicVal.data = (float)batteryRearLogicVoltage / 10.0f;
  pubVoltageRearLogic.publish(&voltageRearLogicVal);

  currentFrontLeftVal.data = (float)currentFrontLeft / 100.0f;
  pubCurrentFrontLeft.publish(&currentFrontLeftVal);

  currentFrontRightVal.data = (float)currentFrontRight / 100.0f;
  pubCurrentFrontRight.publish(&currentFrontRightVal);

  currentRearLeftVal.data = (float)currentRearLeft / 100.0f;
  pubCurrentRearLeft.publish(&currentRearLeftVal);

  currentRearRightVal.data = (float)currentRearRight / 100.0f;
  pubCurrentRearRight.publish(&currentRearRightVal);
}

void loop() {
  if (rosInitialized == true)
    checkTimers();
  //str_msg.data = hello;
  //chatter.publish( &str_msg );
  
  /*GPS.read();

  if (GPS.newNMEAreceived()) 
  {
    if (!GPS.parse(GPS.lastNMEA()))
      return;
  }*/
  nh.spinOnce();
  delay(1);
}
