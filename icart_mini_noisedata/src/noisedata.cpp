#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <sstream>
#include <math.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdlib.h>
#include <time.h>

using namespace Eigen;

class Noisedata{
  public:
    Noisedata(){
      robot_gps_sub = nh.subscribe("/robot_gps",10, &Noisedata::robot_gpsCallBack, this);
      robot_imu_sub = nh.subscribe("/robot_imu",10, &Noisedata::robot_imuCallBack, this);

      state = Vector3f::Zero();

      robot_noise_state_pub = nh.advertise<geometry_msgs::Pose2D>("/robot_pose_added_noise", 1000);

      ros::Rate loop_rate(10);
    }

    void robot_gpsCallBack(const gazebo_msgs::ModelStates::ConstPtr& robot_gps)
    {
      state[0] = robot_gps->pose[1].position.x;
      state[1] = robot_gps->pose[1].position.y;
    }

    void robot_imuCallBack(const sensor_msgs::Imu::ConstPtr& robot_imu)
    {
      state[2] = robot_imu->orientation.z;
    }

    void calc()
    {
      Vector3f noise_state = Vector3f::Zero();

      //calculate
      srand(time(NULL));
      for(int n=0; n<3; n++){
        float num = (float)rand() / 3276700000.0;

        noise_state[n] = state[n] + num;
      }
      ROS_INFO("\n x:%f\n y:%f\n theta:%f\n", noise_state[0], noise_state[1], noise_state[2]);

      //store
      robot_state.x = noise_state[0];
      robot_state.y = noise_state[1];
      robot_state.theta = noise_state[2];

      robot_noise_state_pub.publish(robot_state);
    }

    void run(){
      while(ros::ok()){
        calc();
        ros::spinOnce();
      }
    }

  private:
    ros::NodeHandle nh;

    ros::Subscriber robot_gps_sub;
    ros::Subscriber robot_imu_sub;

    ros::Publisher robot_noise_state_pub;

    geometry_msgs::Pose2D robot_state;

    Vector3f state;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_noisedata");
  Noisedata robot_noise_state;
  robot_noise_state.run();

  return 0;
}
