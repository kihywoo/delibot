#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "delibot_msgs/delibot_odom.h"

nav_msgs::Odometry odom;
geometry_msgs::TransformStamped odom_tf;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //퍼블리쉬 할 토픽 선언
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 100);

    //섭스크라이브 할 토픽픽 선언
    delibot_odom_sub_ = nh_.subscribe("delibot_odom", 100, &SubscribeAndPublish::delibotOdomCallback, this);
  }

  void delibotOdomCallback(const delibot_msgs::delibot_odom& odom_robot)
  {
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_footprint";

        odom.pose.pose.position.x = odom_robot.position.x;
        odom.pose.pose.position.y = odom_robot.position.y;
        odom.pose.pose.position.z = 0;
        odom.pose.pose.orientation = (geometry_msgs::Quaternion)tf::createQuaternionMsgFromYaw(odom_robot.position.z);

        odom.twist.twist.linear.x  = odom_robot.v;
        odom.twist.twist.angular.z = odom_robot.w;

        odom_pub_.publish(odom);

        odom_tf.header = odom.header;
        odom_tf.child_frame_id = odom.child_frame_id;
        odom_tf.transform.translation.x = odom.pose.pose.position.x;
        odom_tf.transform.translation.y = odom.pose.pose.position.y;
        odom_tf.transform.translation.z = odom.pose.pose.position.z;
        odom_tf.transform.rotation      = odom.pose.pose.orientation;
        odom_tf.header.stamp = ros::Time::now();
        tf_broadcaster_.sendTransform(odom_tf);

        ROS_INFO("SUCESS");
  }

private: //private으로 NodeHandle과 publisher, subscriber를 선언한다.
  ros::NodeHandle nh_; 
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber delibot_odom_sub_;
};

int main(int argc, char **argv) // 노드 메인 함수
{
    ros::init(argc, argv, "tf_publisher"); // 노드명 초기화

    SubscribeAndPublish SAPObject; //클래스 객체 선을 하게 되면 모든게 된다.

    ros::spin();

    return 0;
}