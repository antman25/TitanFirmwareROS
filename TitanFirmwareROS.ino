#include "common.h"


void InitializeIMU()
{
  if(!bno.begin())
  {
  
  }

  delay(1000);
 
  bno.setExtCrystalUse(true);
}

void publishDebug()
{
  char output[80];
  
  //sprintf(output,"%i-%i-%i  %i:%i:%i ** %f - %f",GPS.month, GPS.day, GPS.year, GPS.hour, GPS.minute, GPS.seconds,(float)setpointRightVel,(float)motorRightOutput);
  
  //sprintf(output, "Heading: %f - %f - %f\nLeft Setpoint: %f\nLeft Output: %f\nLeftVel: %f\nRight Setpoint: %f\nRight Output: %f\nRightVel: %f\n",  (float)currentHeading, sin(currentHeading / 2.0), cos(currentHeading / 2.0),(float)setpointLeftVel,(float)motorLeftOutput, (float)currentLeftVel, (float)setpointRightVel,(float)motorRightOutput,(float)currentRightVel);
  //sprintf(output,"Left: %f  -- Right: %f\n",motorLeftOutput,motorRightOutput);
  sprintf(output,"%i - %i - %i - %i\n", spFrontLeftMotor, spFrontRightMotor, spRearLeftMotor, spRearRightMotor);
  debug.data = output;
  pubDebug.publish( &debug );
}

void setFrontLeftSpeed(int speed)
{
  roboclaw.SpeedM2(ROBOCLAW_FRONT_ID,speed);
}

/*long getFrontLeftEncoder()
{
  //roboclaw.SpeedM2(ROBOCLAW_FRONT_ID,speed);
}*/

void setFrontRightSpeed(int speed)
{
  roboclaw.SpeedM1(ROBOCLAW_FRONT_ID,speed);
}

void updateMotors()
{
  //0x80, M2 = Front Left
  setFrontLeftSpeed(spFrontLeftMotor);
  
  //0x80, M1 = Front Right
  //roboclaw.SpeedM1(ROBOCLAW_FRONT_ID,spFrontRightMotor);
  setFrontRightSpeed(spFrontRightMotor);
  
  //0x81, M2 = Front Left
  roboclaw.SpeedM2(ROBOCLAW_REAR_ID,spRearLeftMotor);
  
  //0x81, M1 = Front Right
  roboclaw.SpeedM1(ROBOCLAW_REAR_ID,spRearRightMotor);
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
  spRearLeftMotor = (int)-msg.data;
  timerMotorTimeout = millis();
}
void cbRearRightMotorCmd( const std_msgs::Int64 &msg)
{
  spRearRightMotor = (int)msg.data;
  timerMotorTimeout = millis();
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
   
   publishIMU();
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



void setup() {
  roboclaw.begin(57600);
  nh.initNode();
  //broadcaster.init(nh);

  nh.advertise(pubFrontLeftEncoder);
  nh.advertise(pubFrontRightEncoder);
  nh.advertise(pubRearLeftEncoder);
  nh.advertise(pubRearRightEncoder);
  
  nh.advertise(pubIMU);
  nh.advertise(pubMag);
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
  
  InitializeIMU();
  rosInitialized = true;
}


void publishIMU()
{
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE );
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
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
  imuMsg.orientation.z = quat.z();
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

  magMsg.magnetic_field.x = mag.x() / 1000000.0;
  magMsg.magnetic_field.y = mag.y() / 1000000.0;
  magMsg.magnetic_field.z = mag.z()/ 1000000.0;

  pubMag.publish(&magMsg);
}


void updateEncoders()
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;  
  int32_t encFrontLeft = roboclaw.ReadEncM2(ROBOCLAW_FRONT_ID, &status1, &valid1);
  int32_t encFrontRight = roboclaw.ReadEncM1(ROBOCLAW_FRONT_ID, &status2, &valid2);
  int32_t encRearLeft = roboclaw.ReadEncM2(ROBOCLAW_REAR_ID, &status3, &valid3);
  int32_t encRearRight = roboclaw.ReadEncM1(ROBOCLAW_REAR_ID, &status4, &valid4);

  encoderFrontLeftVal.data = encFrontLeft;
  pubFrontLeftEncoder.publish(&encoderFrontLeftVal);

  encoderFrontRightVal.data = encFrontRight;
  pubFrontRightEncoder.publish(&encoderFrontRightVal);

  encoderRearLeftVal.data = -encRearLeft;
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

  nh.spinOnce();
  //delay(1);
}
