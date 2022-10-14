#include "core_config.h"

void setup() 
{
  /*MCU 기능 시작*/
  avr.init();
  sensors.init();
}

void loop() 
{
  uint32_t t = millis();

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  { 
    String arr(rxBuf);
    int index[2] = {arr.indexOf("b"), arr.indexOf("c")};
    //index[0] = arr.indexOf("b");
    //index[1] = arr.indexOf("c");
    //String receiveData1 = arr.substring(1, index[0]);
    //String receiveData2 = arr.substring(index[0]+1, index[1]);
    String receiveData[2] = {arr.substring(1, index[0]), arr.substring(index[0]+1, index[1])};
    
    goal_velocity_from_cmd[LINEAR] = receiveData[0].toFloat();
    goal_velocity_from_cmd[ANGULAR] = receiveData[1].toFloat();
    
    avr.controlMotor(goal_velocity_from_cmd);

    tTime[0] = t;
  }

  if ((t-tTime[1]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))
  {
    String packet[6] = {"a", "b", "c", "d", "e", "f"};
    String arr = "";
    char sendData[50];
    
    updateOdom();

    for(int i =0; i<5; i++)
    {
      arr = arr + packet[i] + String(odom_info[i],2);
    }
    arr = arr + packet[5];

    arr.toCharArray(sendData, arr.length()+1);
    
    serialWritelnNoInterrup(sendData);

    tTime[1] = t;
  }

  // Update the IMU unit
  sensors.updateIMU();
}

/*******************************************************************************
* Update odom information
*******************************************************************************/
void updateOdom(void)
{
  unsigned long time_now = millis();
  static unsigned long prev_update_time = 0;
  unsigned long step_time = time_now - prev_update_time;
  prev_update_time = time_now;

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));
}


/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;
  
  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;
  step_time = 0.0;

  avr.readEncoderTick(last_diff_tick[LEFT], last_diff_tick[RIGHT]);
  avr.clearEncoderTick();
  
  step_time = diff_time;

  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  delta_s     = RADIUS * (wheel_r + wheel_l) / 2.0;
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  odom_info[0] += delta_s * cos(odom_info[2] + (delta_theta / 2.0));
  odom_info[1] += delta_s * sin(odom_info[2] + (delta_theta / 2.0));
  odom_info[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v = delta_s / step_time;
  w = delta_theta / step_time;

  odom_info[3] = v;
  odom_info[4] = w;

  last_theta = theta;

  return true;
}
