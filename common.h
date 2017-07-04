#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GPS.h>
#include <utility/imumaths.h>

#include <ros.h>
#include "ros.h"
#include <ros/time.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

 #include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h> 

#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include "RoboClaw.h"
#include "HWSerial.h"

#include <Servo.h> 
 
Servo servo1;  // create servo object to control a servo 
Servo servo2;                
Servo servo3;
 

ros::NodeHandle nh;
//ros::NodeHandle_<HWSerial> nh_bluetooth;


sensor_msgs::Imu imuRawMsg;
sensor_msgs::Imu imuFilteredMsg;
sensor_msgs::MagneticField magMsg;
sensor_msgs::Temperature tempMsg;

std_msgs::String debug;

std_msgs::Int64 encoderFrontLeftVal;
std_msgs::Int64 encoderFrontRightVal;
std_msgs::Int64 encoderRearLeftVal;
std_msgs::Int64 encoderRearRightVal;

std_msgs::Float32 voltageFrontLogicVal;
std_msgs::Float32 voltageRearLogicVal;

std_msgs::Float32 voltageFrontMainVal;
std_msgs::Float32 voltageRearMainVal;

std_msgs::Float32 currentFrontLeftVal;
std_msgs::Float32 currentFrontRightVal;
std_msgs::Float32 currentRearLeftVal;
std_msgs::Float32 currentRearRightVal;

ros::Publisher pubIMURaw("imu/raw", &imuRawMsg);
ros::Publisher pubIMUFiltered("imu/data", &imuFilteredMsg);
ros::Publisher pubMag("imu/mag", &magMsg);
ros::Publisher pubTemp("imu/temp", &tempMsg);

ros::Publisher pubDebug("debug", &debug);


ros::Publisher pubFrontLeftEncoder("encoder_fl_wheel", &encoderFrontLeftVal);
ros::Publisher pubFrontRightEncoder("encoder_fr_wheel", &encoderFrontRightVal);
ros::Publisher pubRearLeftEncoder("encoder_rl_wheel", &encoderRearLeftVal);
ros::Publisher pubRearRightEncoder("encoder_rr_wheel", &encoderRearRightVal);

ros::Publisher pubVoltageFrontMain("voltage_front_main", &voltageFrontMainVal);
ros::Publisher pubVoltageRearMain("voltage_rear_main", &voltageRearMainVal);

ros::Publisher pubVoltageFrontLogic("voltage_front_logic", &voltageFrontLogicVal);
ros::Publisher pubVoltageRearLogic("voltage_rear_logic", &voltageRearLogicVal);

ros::Publisher pubCurrentFrontLeft("current_front_left", &currentFrontLeftVal);
ros::Publisher pubCurrentFrontRight("current_front_right", &currentFrontRightVal);
ros::Publisher pubCurrentRearLeft("current_rear_left", &currentRearLeftVal);
ros::Publisher pubCurrentRearRight("current_rear_right", &currentRearRightVal);



Adafruit_BNO055 bno = Adafruit_BNO055();
RoboClaw roboclaw(&Serial1,10000);
//Adafruit_GPS GPS(&Serial3);

long seq = 0;
bool rosInitialized = false;


#define UPDATE_RATE_IMU           50
#define UPDATE_RATE_ENCODER       50
#define UPDATE_RATE_DEBUG         1000
#define UPDATE_RATE_MOTOR         50
#define UPDATE_RATE_MOTOR_INFO    200

#define TIMEOUT_MOTOR_CMD         1000

#define ROBOCLAW_FRONT_ID         0x81
#define ROBOCLAW_REAR_ID          0x80

#define MOTOR_KP                  6400
#define MOTOR_KI                  2200
#define MOTOR_KD                  0
#define MOTOR_QPPS                6400 

/*
 * Motor 
KP = 6400.00
KI = 2200.00
KD = 0.00
qpps = 6400

 */


long timerIMU = millis();
long timerDebug = millis();
long timerEncoder = millis();
long timerMotorUpdate = millis();
long timerMotorTimeout = millis();
long timerMotorInfo = millis();

int spFrontLeftMotor = 0;
int spFrontRightMotor = 0;
int spRearLeftMotor = 0;
int spRearRightMotor = 0;

float spServo1 = 0.0f;
float spServo2 = 0.0f;
float spServo3 = 0.0f;


void InitializeIMU();
void InitializeGPS();
void publishDebug();

