#ifndef DELIBOT_CORE_CONFIG_H_
#define DELIBOT_CORE_CONFIG_H_

#include <avr.h>
#include <sensor.h>
#include <IMU.h>
#include <math.h>
#include <string.h>

#define CONTROL_MOTOR_SPEED_FREQUENCY          20
#define DRIVE_INFORMATION_PUBLISH_FREQUENCY    30

#define WHEEL_NUM                              2

// Callback function prototypes

// Function prototypes
void updateOdom(void);
bool calcOdometry(double diff_time);

static uint32_t tTime[3];
float goal_velocity_from_cmd[WHEEL_NUM] = {0.0, 0.0};
float odom_info[5];
int last_diff_tick[WHEEL_NUM] = {0, 0};
extern char rxBuf[BUF_SIZE];  //전송할 데이터를 저장하는 버퍼

MCU avr;
Sensor sensors;

#endif
