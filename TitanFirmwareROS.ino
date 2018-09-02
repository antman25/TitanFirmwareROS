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

void publishDebug()
{
  char output[80];
  sprintf(output,"Encoder: %i - Setpoint: %i",(int)currentEncoder,(int)setpointEncoder);
  //sprintf(output,"%i-%i-%i  %i:%i:%i ** %f - %f",GPS.month, GPS.day, GPS.year, GPS.hour, GPS.minute, GPS.seconds,(float)GPS.lat,(float)GPS.lon);
  
  //sprintf(output, "Heading: %f - %f - %f\nLeft Setpoint: %f\nLeft Output: %f\nLeftVel: %f\nRight Setpoint: %f\nRight Output: %f\nRightVel: %f\n",  (float)currentHeading, sin(currentHeading / 2.0), cos(currentHeading / 2.0),(float)setpointLeftVel,(float)motorLeftOutput, (float)currentLeftVel, (float)setpointRightVel,(float)motorRightOutput,(float)currentRightVel);
  //sprintf(output,"Left: %f  -- Right: %f\n",motorLeftOutput,motorRightOutput);
  //sprintf(output,"%i - %i - %i - %i\n", spFrontLeftMotor, spFrontRightMotor, spRearLeftMotor, spRearRightMotor);
  debug.data = output;
  pubDebug.publish( &debug );
}

void publishMotorStatus()
{
  arm_status.z_axis_pos = currentEncoder * Z_AXIS_M_PER_TICK;
  arm_status.servo1_pos = curServo1;
  arm_status.servo2_pos = curServo2;
  arm_status.servo3_pos = curServo3;
  arm_status.pump_state = pump_state;
  pubArmStatus.publish( &arm_status );
}


void cbArmCmd( const titan_msgs::ArmCmd &msg)
{
  setpointEncoder = (int)(msg.z_axis_pos * Z_AXIS_TICKS_PER_M);
  
  spServo1 = msg.servo1_pos;
  spServo2 = msg.servo2_pos;
  spServo3 = msg.servo3_pos;
  
  //pump_state = msg.pump_state;
  if (msg.pump_state)
  {
    pump_state = true;
  }
  else
  {
    pump_state = false;
  }
  
  

  if (setpointEncoder < 0)
  {
    setpointEncoder = 0;
  }

  if (setpointEncoder > 20000)
  {
    setpointEncoder = 20000;
  }
}

void cbMotorStatus( const titan_msgs::Status &msg)
{
  battery_voltage = (float)msg.BatteryV;
}

void updateStatusLED()
{
  

  if (battery_voltage >= BAT_80)
  {
    setStatusLED(0,64,0);
  }
  else if (battery_voltage < BAT_80 && battery_voltage >= BAT_60)
  {
    setStatusLED(0,64,64);
  }
  else if (battery_voltage < BAT_60 && battery_voltage >= BAT_40)
  {
    setStatusLED(0,64,64);
  }
  else if (battery_voltage < BAT_40)
  {
    setStatusLED(255,0,0);
  }
}

void updateMotor()
{
  uint8_t status1;
  bool valid1;  
  int32_t ret = roboclaw.ReadEncM1(ROBOCLAW_ID, &status1, &valid1);

  if (valid1 == true)
  {
    currentEncoder = ret;
  }

  
  if (currentEncoder <= setpointEncoder - ENC_TOLERANCE)
  {
    float duty = (0.3 * 32678.0);
    roboclaw.DutyM1(ROBOCLAW_ID, (int16_t)duty);
  }
  else if (currentEncoder >= setpointEncoder + ENC_TOLERANCE)
  {
    float duty = -(0.3 * 32678.0);
    roboclaw.DutyM1(ROBOCLAW_ID, (int16_t)duty);
  }
  else
  {
    roboclaw.DutyM1(ROBOCLAW_ID, 0);
  }

  servo1.write(getAngleTime1(spServo1)); 
  servo2.write(getAngleTime1(spServo2)); 
  servo3.write(getAngleTime1(spServo3));

  curServo1 = spServo1;
  curServo2 = spServo2;
  curServo3 = spServo3;

  /*if (pump_state == true)
  {
    digitalWrite(PIN_PUMPS, HIGH);
  }
  else
  {
    digitalWrite(PIN_PUMPS, LOW);
  }*/
}

void checkTimers()
{

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

  if (millis() -timerRefreshMotor > UPDATE_RATE_MOTOR)
  {
    timerRefreshMotor = millis();
    updateMotor();
  }

  if (millis() - timerMotorStatus > UPDATE_RATE_MOTOR_STATUS)
  {
    timerMotorStatus = millis();
    publishMotorStatus();
  }
  
}

//ros::Subscriber<std_msgs::Float32> subServo1Cmd("servo1_angle_cmd", cbServo1Cmd);
//ros::Subscriber<std_msgs::Float32> subServo2Cmd("servo2_angle_cmd", cbServo2Cmd);
//ros::Subscriber<std_msgs::Float32> subServo3Cmd("servo3_angle_cmd", cbServo3Cmd);
//ros::Subscriber<std_msgs::Float32> subZAxisCmd("z_axis_cmd", cbZAxisCmd);
ros::Subscriber<titan_msgs::ArmCmd> subArmCmd("arm_cmd", cbArmCmd);

ros::Subscriber<titan_msgs::Status> subMotorStatus("motor_status", cbMotorStatus);

void setStatusLED(byte red, byte green, byte blue)
{
  analogWrite(PIN_LED_RED, 255-red); 
  analogWrite(PIN_LED_GREEN, 255-green); 
  analogWrite(PIN_LED_BLUE, 255-blue); 
}

void setup() {
  roboclaw.begin(115200);
  
  //InitializeIMU();
  ina219.begin();
  
  servo1.attach(PIN_PWM_1,850,2350); 
  servo2.attach(PIN_PWM_2,850,2350);
  servo3.attach(PIN_PWM_3,850,2350);
  servo4.attach(PIN_PWM_4,850,2350);

  servo1.write(getAngleTime1(0)); 
  servo2.write(getAngleTime2(0));
  servo3.write(getAngleTime3(0));

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  //pinMode(PIN_PUMPS, OUTPUT);

  
  setStatusLED(64,64,64);
  
  nh.initNode();

  
  nh.advertise(pubDebug);
  nh.advertise(pubArmStatus);

  nh.subscribe(subArmCmd);
  nh.subscribe(subMotorStatus);
  
  rosInitialized = true;
}





void loop() {
  if (rosInitialized == true)
    checkTimers();
  nh.spinOnce();
}


