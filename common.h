#include <Wire.h>

#include "RoboClaw.h"

#include <ros.h>
#include "ros.h"
#include <ros/time.h>
#include <Adafruit_INA219.h>


#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <titan_msgs/Status.h>
#include <titan_msgs/ArmCmd.h>
#include <titan_msgs/ArmStatus.h>

#include <Servo.h> 

RoboClaw roboclaw(&Serial1,10000);
 
Servo servo1;  // create servo object to control a servo 
Servo servo2;                
Servo servo3;
Servo servo4; 

ros::NodeHandle nh;

float battery_voltage=0.0f;

std_msgs::String debug;
titan_msgs::ArmStatus arm_status;

ros::Publisher pubDebug("debug", &debug);
ros::Publisher pubArmStatus("arm_status", &arm_status);

//Adafruit_BNO055 bno = Adafruit_BNO055();

long seq = 0;
bool rosInitialized = false;
bool pump_state = false;



#define UPDATE_RATE_DEBUG         1000
#define UPDATE_RATE_STATUS_LED    50
#define UPDATE_RATE_MOTOR         50
#define UPDATE_RATE_MOTOR_STATUS  50


#define PIN_LED_RED                3 
#define PIN_LED_GREEN              4
#define PIN_LED_BLUE               5
#define PIN_PWM_1                  20
#define PIN_PWM_2                  21
#define PIN_PWM_3                  22
#define PIN_PWM_4                  23


#define BAT_CELLS                 6
#define BAT_FULL                  2.2 * BAT_CELLS
#define BAT_EMPTY                 1.9 * BAT_CELLS


#define BAT_SLOPE                 (BAT_FULL-BAT_EMPTY)/100.0
#define BAT_100                   BAT_FULL
#define BAT_80                    (80.0 * BAT_SLOPE) + BAT_EMPTY      
#define BAT_60                    (60.0 * BAT_SLOPE) + BAT_EMPTY   
#define BAT_40                    (40.0 * BAT_SLOPE) + BAT_EMPTY   
#define BAT_20                    (20.0 * BAT_SLOPE) + BAT_EMPTY   
#define BAT_0                     BAT_EMPTY

#define ROBOCLAW_ID               128

#define ENC_TOLERANCE             50

#define PIN_PUMPS                 2


#define Z_AXIS_TICKS_PER_M        1284.0 / 0.008
#define Z_AXIS_M_PER_TICK        0.008 / 1284.0
/*
 * Motor 
KP = 6400.00
KI = 2200.00
KD = 0.00
qpps = 6400

 */

Adafruit_INA219 ina219;

long timerDebug = millis();
long timerStatusLED = millis();
long timerRefreshMotor = millis();
long timerMotorStatus = millis();

float spServo1 = 0.0f;
float spServo2 = 0.0f;
float spServo3 = 0.0f;


float curServo1 = 0.0f;
float curServo2 = 0.0f;
float curServo3 = 0.0f;


int32_t currentEncoder = 0;
int32_t setpointEncoder = 0;
void publishDebug();
void publishMotorStatus();
