#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <Encoder.h>


#include <ros.h>
#include <ros/time.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <sensor_msgs/Imu.h>

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

#define PIN_ENCODER_LF_A  20
#define PIN_ENCODER_LF_B  21

#define PIN_ENCODER_LR_A  22
#define PIN_ENCODER_LR_B  23

#define PIN_ENCODER_RF_A  14
#define PIN_ENCODER_RF_B  15

#define PIN_ENCODER_RR_A  16
#define PIN_ENCODER_RR_B  17


Encoder encoderLF(PIN_ENCODER_LF_A, PIN_ENCODER_LF_B); 
Encoder encoderLR(PIN_ENCODER_LR_A, PIN_ENCODER_LR_B); 

Encoder encoderRF(PIN_ENCODER_RF_A, PIN_ENCODER_RF_B); 
Encoder encoderRR(PIN_ENCODER_RR_A, PIN_ENCODER_RR_B);

ros::NodeHandle nh;
sensor_msgs::Imu imuMsg;

std_msgs::String debug;

std_msgs::Int64 encoderLeftVal;
std_msgs::Int64 encoderRightVal;

ros::Publisher pubIMU("imu_data", &imuMsg);
ros::Publisher pubDebug("debug", &debug);


ros::Publisher pubLeftEncoder("lwheel_encoder", &encoderLeftVal);
ros::Publisher pubRightEncoder("rwheel_encoder", &encoderRightVal);


Adafruit_BNO055 bno = Adafruit_BNO055();

#define SabertoothSerial      Serial1
#define SABERTOOTH_ADDR       128

#define SABERTOOTH_CMD_MOTOR1_FWD   1
#define SABERTOOTH_CMD_MOTOR1_REV   0

#define SABERTOOTH_CMD_MOTOR2_FWD   5
#define SABERTOOTH_CMD_MOTOR2_REV   4

#define SABERTOOTH_CMD_MIN_VOLTAGE  2
#define SABERTOOTH_CMD_TIMEOUT      14

#define CMD_TIMEOUT_VAL     10
#define MIN_VOLTAGE         10

#define MIN_VOLTAGE_VAL     (int)((MIN_VOLTAGE - 6.0) * 5.0)

long seq = 0;
bool rosInitialized = false;



#define UPDATE_RATE_IMU       50
#define UPDATE_RATE_ENCODER   50
#define UPDATE_RATE_DEBUG     1000
#define UPDATE_RATE_MOTOR     100

#define TIMEOUT_MOTOR_CMD     1000

long timerIMU = millis();
long timerDebug = millis();
long timerEncoder = millis();
long timerMotorUpdate = millis();
long timerMotorTimeout = millis();



double motorLeftOutput = 0.0F;
double motorRightOutput = 0.0F;

