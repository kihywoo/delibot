#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "delibot_msgs/drive_info.h"
#include "geometry_msgs/Quaternion.h"
#include <math.h>
#include <stdio.h>

#define LEFT 0
#define RIGHT 1
#define RADIUS 0.075
#define TICK2RAD 1.59e-3

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //퍼블리쉬 할 토픽 선언
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1024);

    //섭스크라이브 할 토픽픽 선언
    delibot_info_sub_ = nh_.subscribe("delibot_info", 1024, &SubscribeAndPublish::delibotInfoCallback, this);
  }

  void delibotInfoCallback(const delibot_msgs::drive_info& info)
  {
        calcOdometry(info.left, info.right, 0.05, info.quat);
        updateOdom(pose, vel);    
  }

  void calcOdometry(int left, int right, float time, geometry_msgs::Quaternion orientation)
  {
        double wheel_l, wheel_r;
        double delta_s, theta, delta_theta;
        static double last_theta = 0.0;

        double l = left;
        double r = right;

        wheel_l = wheel_r = 0.0;
        delta_s = delta_theta = theta = 0.0;

        wheel_l = TICK2RAD * l;
        wheel_r = TICK2RAD * r;

        delta_s = RADIUS * (wheel_r + wheel_l) / 2.0;
        theta = atan2f(orientation.y*orientation.z + orientation.x*orientation.w, 
                0.5f - orientation.z*orientation.z - orientation.w*orientation.w);
        delta_theta = theta - last_theta;

        pose[0] += delta_s * cos(pose[2] + (delta_theta / 2.0));
        pose[1] += delta_s * sin(pose[2] + (delta_theta / 2.0));
        pose[2] += delta_theta;

        vel[0] = delta_s / time;
        vel[1] = delta_theta / time;
        last_theta = theta;
  }

  void updateOdom(float * p, float * v)
  {
      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "odom";
      odom.child_frame_id  = "base_footprint";
      odom.pose.pose.position.x = p[0];
      odom.pose.pose.position.y = p[1];
      odom.pose.pose.position.z = 0;
      odom.pose.pose.orientation = (geometry_msgs::Quaternion)tf::createQuaternionMsgFromYaw(p[2]);

      odom.twist.twist.linear.x  = v[0];
      odom.twist.twist.angular.z = v[1];
      odom_pub_.publish(odom);

      odom_tf.header = odom.header;
      odom_tf.child_frame_id = odom.child_frame_id;
      odom_tf.header.stamp = ros::Time::now();
      odom_tf.transform.translation.x = odom.pose.pose.position.x;
      odom_tf.transform.translation.y = odom.pose.pose.position.y;
      odom_tf.transform.translation.z = odom.pose.pose.position.z;
      odom_tf.transform.rotation      = odom.pose.pose.orientation;
      tf_broadcaster_.sendTransform(odom_tf);
  }

private: //private으로 NodeHandle과 publisher, subscriber를 선언한다.
  ros::NodeHandle nh_; 
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber delibot_info_sub_;

  nav_msgs::Odometry odom;
  geometry_msgs::TransformStamped odom_tf;

  float pose[3] = {0,0,0};
  float vel[2]= {0,0};;
};

int main(int argc, char **argv) // 노드 메인 함수
{
    ros::init(argc, argv, "test_publisher"); // 노드명 초기화

    SubscribeAndPublish SAPObject; //클래스 객체 선을 하게 되면 모든게 된다.

    ros::spin();

    return 0;
}