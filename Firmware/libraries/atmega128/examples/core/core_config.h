#ifndef DELIBOT_CORE_CONFIG_H_
#define DELIBOT_CORE_CONFIG_H_

#include <avr.h>
#include <sensor.h>
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <delibot_msgs/delibot_odom.h>
#include <delibot_msgs/PIgain.h>
#include <math.h>

#define CONTROL_MOTOR_SPEED_FREQUENCY          30
#define IMU_PUBLISH_FREQUENCY                  200
#define GAIN_PUBLISH_FREQUENCY                 1
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30

#define WHEEL_NUM                              2

// Callback function prototypes
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
void controlGainCallback(const delibot_msgs::PIgain& control_msg);

// Function prototypes
void publishDriveInformation(void);
void publishImuMsg(void);
//void publishMagMsg(void);

ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateTime(void);
void updateOdometry(void);
void updateJointStates(void);
void updateGyroCali(bool isConnected);

void initJointStates(void);
void initOdom(void);

bool calcOdometry(double diff_time);
void printState();

/*******************************************************************************
* ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;


/*******************************************************************************
* Subscriber
*******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);
ros::Subscriber<delibot_msgs::PIgain> control_gain_sub("pi", controlGainCallback);


/*******************************************************************************
* Publisher
*******************************************************************************/
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Magnetic field
/*sensor_msgs::MagneticField mag_msg;
ros::Publisher mag_pub("magnetic_field", &mag_msg);*/

// Joint(Dynamixel) state of Turtlebot3
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

delibot_msgs::delibot_odom odom;
ros::Publisher odom_pub("delibot_odom", &odom);

/*******************************************************************************
* SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[4];

/*******************************************************************************
* Declaration for controllers
*******************************************************************************/
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};

/*******************************************************************************
* Declaration for SLAM and navigation
*******************************************************************************/
unsigned long prev_update_time;
float odom_pose[3];
float odom_vel[3];

/*******************************************************************************
* Calculation for odometry
*******************************************************************************/
int last_diff_tick[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};

/*******************************************************************************
* Update Joint State
*******************************************************************************/
double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};
char array[10];

MCU avr;
Sensor sensors;

#endif
