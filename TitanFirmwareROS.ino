#include "common.h"

int getAngleTime1(double angle)
{
  return map(angle, -180, 180, 900, 2250);     // scale it to use it with the servo (value between 0 and 180) 
}

int getAngleTime2(double angle)
{
  return map(angle, -90, 90, 850, 2250);     // scale it to use it with the servo (value between 0 and 180) 
}

int getAngleTime3(double angle)
{
  return map(angle, -90, 90, 850, 2200);     // scale it to use it with the servo (value between 0 and 180) 
}

void InitializeIMU()
{
    int status = IMU.begin();
    if (status < 0) {
      while(1) {
        setStatusLED(0,64,0);
        delay(250);
        setStatusLED(64,64,0);
        delay(250);
        
      }
    }
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
  roboclaw.SpeedM2(ROBOCLAW_FRONT_ID,speed);
}

void setFrontRightSpeed(int speed)
{
  roboclaw.SpeedM1(ROBOCLAW_FRONT_ID,speed);
}

void setRearLeftSpeed(int speed)
{
  roboclaw.SpeedM2(ROBOCLAW_REAR_ID,speed);
}

void setRearRightSpeed(int speed)
{
  roboclaw.SpeedM1(ROBOCLAW_REAR_ID,speed);
}


void updateMotors()
{
  if (INVERT_MOTORS == 0)
  {
    setFrontLeftSpeed(spFrontLeftMotor);
    setFrontRightSpeed(spFrontRightMotor);
    setRearLeftSpeed(spRearLeftMotor);
    setRearRightSpeed(spRearRightMotor);
  }
  else
  {
    setFrontLeftSpeed(-spFrontLeftMotor);
    setFrontRightSpeed(-spFrontRightMotor);
    setRearLeftSpeed(-spRearLeftMotor);
    setRearRightSpeed(-spRearRightMotor);
  }
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

void updateStatusLED()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  if (busvoltage >= BAT_80)
  {
    setStatusLED(0,64,0);
  }
  else if (busvoltage < BAT_80 && busvoltage >= BAT_60)
  {
    setStatusLED(0,64,64);
  }
  else if (busvoltage < BAT_60 && busvoltage >= BAT_40)
  {
    setStatusLED(0,64,64);
  }
  else if (busvoltage < BAT_40)
  {
    setStatusLED(255,0,0);
  }
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

  if (millis() - timerStatusLED > UPDATE_RATE_STATUS_LED)
  {
    timerStatusLED = millis();
    updateStatusLED();
  }
  
}


ros::Subscriber<std_msgs::Int64> subFrontLeftMotorCmd("vel_sp_fl_wheel", cbFrontLeftMotorCmd);
ros::Subscriber<std_msgs::Int64> subFrontRightMotorCmd("vel_sp_fr_wheel", cbFrontRightMotorCmd);
ros::Subscriber<std_msgs::Int64> subRearLeftMotorCmd("vel_sp_rl_wheel", cbRearLeftMotorCmd);
ros::Subscriber<std_msgs::Int64> subRearRightMotorCmd("vel_sp_rr_wheel", cbRearRightMotorCmd);

ros::Subscriber<std_msgs::Float32> subServo1Cmd("servo1_angle_cmd", cbServo1Cmd);
ros::Subscriber<std_msgs::Float32> subServo2Cmd("servo2_angle_cmd", cbServo2Cmd);
ros::Subscriber<std_msgs::Float32> subServo3Cmd("servo3_angle_cmd", cbServo3Cmd);

void setStatusLED(byte red, byte green, byte blue)
{
  analogWrite(PIN_LED_RED, 255-red); 
  analogWrite(PIN_LED_GREEN, 255-green); 
  analogWrite(PIN_LED_BLUE, 255-blue); 
}

void setup() {
  InitializeIMU();
  ina219.begin();
  
  servo1.attach(PIN_PWM_1,850,2350); 
  servo2.attach(PIN_PWM_2,850,2350);
  servo3.attach(PIN_PWM_3,850,2350);
  servo4.attach(PIN_PWM_4,850,2350);

  servo1.write(getAngleTime1(0)); 
  servo2.write(getAngleTime2(0));
  servo3.write(getAngleTime3(0));
  servo4.write(getAngleTime3(0));

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  setStatusLED(0,0,64);
  
  
  roboclaw.begin(57600);
 
  nh.initNode();

   
  //nh_bluetooth.initNode();

  //nh_bluetooth.advertise(chatter);
  
  nh.advertise(pubFrontLeftEncoder);
  nh.advertise(pubFrontRightEncoder);
  nh.advertise(pubRearLeftEncoder);
  nh.advertise(pubRearRightEncoder);
  
  nh.advertise(pubIMURaw);
  //nh.advertise(pubIMUFiltered);
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
  
  
  
  //InitializeGPS();
  rosInitialized = true;
}



void publishIMU()
{
  IMU.readSensor();
  
  char id[] = "imu_link";
  imuRawMsg.header.frame_id = id;
  imuRawMsg.header.stamp=nh.now();
  imuRawMsg.header.seq = seq;
  seq = seq + 1;

  imuRawMsg.orientation.x = 0;
  imuRawMsg.orientation.y = 0;
  imuRawMsg.orientation.z = 0;
  imuRawMsg.orientation.w = 0;  
  imuRawMsg.orientation_covariance[0] = -1;

  imuRawMsg.linear_acceleration.x = IMU.getAccelY_mss();
  imuRawMsg.linear_acceleration.y = IMU.getAccelX_mss();
  imuRawMsg.linear_acceleration.z = -IMU.getAccelZ_mss();
  imuRawMsg.linear_acceleration_covariance[0] = -1;
 
  imuRawMsg.angular_velocity.x = IMU.getGyroY_rads();
  imuRawMsg.angular_velocity.y = IMU.getGyroX_rads();
  imuRawMsg.angular_velocity.z = -IMU.getGyroZ_rads();
  imuRawMsg.angular_velocity_covariance[0] = -1;

  /*imuRawMsg.linear_acceleration.x = IMU.getAccelX_mss();
  imuRawMsg.linear_acceleration.y = IMU.getAccelY_mss();
  imuRawMsg.linear_acceleration.z = IMU.getAccelZ_mss();
  imuRawMsg.linear_acceleration_covariance[0] = -1;
 
  imuRawMsg.angular_velocity.x = IMU.getGyroY_rads();
  imuRawMsg.angular_velocity.y = IMU.getGyroX_rads();
  imuRawMsg.angular_velocity.z = IMU.getGyroZ_rads();
  imuRawMsg.angular_velocity_covariance[0] = -1;*/
  
  pubIMURaw.publish(&imuRawMsg);

  magMsg.header.frame_id = id;
  magMsg.header.stamp=nh.now();
  magMsg.header.seq = seq;
  seq = seq + 1;
  magMsg.magnetic_field.x = IMU.getMagY_uT() / 1000000.0;
  magMsg.magnetic_field.y = IMU.getMagX_uT() / 1000000.0;
  magMsg.magnetic_field.z = IMU.getMagZ_uT()/ 1000000.0;

  pubMag.publish(&magMsg);


  tempMsg.header.frame_id = id;
  tempMsg.header.stamp=nh.now();
  tempMsg.header.seq = seq;
  seq = seq + 1;
  tempMsg.temperature = IMU.getTemperature_C();
  
  pubTemp.publish(&tempMsg);
  
  /*imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> linear_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE );
  imu::Quaternion quat = bno.getQuat();
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  int8_t temp = bno.getTemp();


  

  
  
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


  */
  
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
  //delay(1);
}

/*void loop()
{
  // servo 1 = right (-) / left(+)
  //servo1.write(getAngleTime1(45)); 

  // servo 2 = up(-)/down(+)
  //servo2.write(getAngleTime2(0)); 

  for (float x = -45;x < 45.0;x += 1.0)
  {
    for (float y = -45.0;y < 45.0;y += 1.0)
    {
      servo1.write(getAngleTime1(x)); 
      servo2.write(getAngleTime2(y)); 
      delay(100);
    }
  }
}*/

