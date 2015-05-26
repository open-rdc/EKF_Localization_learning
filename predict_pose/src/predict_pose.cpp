#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <sstream>
#include <iostream>
#include <math.h>
#include <complex>

using namespace Eigen;

class PredictPose{
public:
  PredictPose(){
    old_est_sigma_sub = nh.subscribe(
      "/robot_pose_convriance",10,&PredictPose::oldSigmaCallBack, this
    );
    old_est_pose_sub = nh.subscribe(
      "/robot_pose_estimate",10,&PredictPose::oldEstPoseCallBack, this
    );
    cmd_vel_sub = nh.subscribe(
      "/icart_mini/cmd_vel",10,&PredictPose::cmd_velCallBack, this
    );

    old = ros::Time::now();
    now = ros::Time::now();

    old_est_pose = Vector3f::Zero();
    alpha = Vector4f::Zero();
    old_sigma = MatrixXd::Zero(3,3);

    est_sigma.data.resize(9);

    est_pose_pub = nh.advertise<geometry_msgs::Pose2D>("/predict_pose",100);
    est_sigma_pub = nh.advertise<std_msgs::Float64MultiArray>("/predict_pose_convariance",100);

    ros::Rate loop_rate(0.1);
  }

  void oldSigmaCallBack(
    const std_msgs::Float64MultiArray::ConstPtr& old_sigma_ptr
  ){
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
        old_sigma(i,j) = old_sigma_ptr->data[i+j];
      }
    }
  }

  void oldEstPoseCallBack(
    const geometry_msgs::Pose2D::ConstPtr& old_est_pose_ptr
  ){
    old_est_pose[0] = old_est_pose_ptr->x;
    old_est_pose[1] = old_est_pose_ptr->y;
    old_est_pose[2] = old_est_pose_ptr->theta;
  }

  void cmd_velCallBack(
    const geometry_msgs::Twist::ConstPtr& cmd_vel_ptr
  ){
    now = ros::Time::now();
    linear = cmd_vel_ptr->linear.x;
    angular = cmd_vel_ptr->angular.z;
  }

  void calc(){
    alpha << 0.0f,0.0f,0.0f,0.0f;

    MatrixXd G = MatrixXd::Zero(3,3);
    MatrixXd V = MatrixXd::Zero(3,2);
    MatrixXd M = MatrixXd::Zero(2,2);
    MatrixXd sigma = MatrixXd::Zero(3,3);

    double delta_t = now.toSec() - old.toSec();
    double speed_per_angular;
    double cos_theta = cos(old_est_pose[2]);
    double sin_theta = sin(old_est_pose[2]);
    double cos_theta_d = cos(old_est_pose[2] + angular * delta_t);
    double sin_theta_d = sin(old_est_pose[2] + angular * delta_t);

    double angular_square = angular * angular;
    double linear_square = linear * linear;

    if( angular != 0 ){
      speed_per_angular = linear / angular;
    }
    else{
      speed_per_angular = 0;
    }
    
    G <<
    1,  0,  -(speed_per_angular) * cos_theta + speed_per_angular * cos_theta_d,
    0,  1,  -(speed_per_angular) * sin_theta + speed_per_angular * sin_theta_d,
    0,  0,  1;

    V <<
    (-sin_theta+sin_theta_d)/angular,  linear*(sin_theta-sin_theta_d)/angular_square + linear*(cos_theta_d*delta_t/angular),
    (cos_theta-cos_theta_d)/angular,   -linear*(cos_theta-cos_theta_d)/angular_square + linear*(sin_theta_d*delta_t/angular),
    0,                                 delta_t;

    M <<
    alpha[0]*linear_square + alpha[1]*angular_square, 0,
    0,                                                alpha[2]*linear_square + alpha[3]*angular_square;

    est_pose.x = old_est_pose[0];
    est_pose.y = old_est_pose[1];
    est_pose.theta = old_est_pose[2];

    est_pose.x += -(speed_per_angular) * sin_theta + speed_per_angular * sin_theta_d;
    est_pose.y += -(speed_per_angular) * cos_theta + speed_per_angular * cos_theta_d;
    est_pose.theta += angular * delta_t;

    sigma = G * old_sigma * G.transpose() + V * M * V.transpose();

    est_pose_pub.publish(est_pose);

    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
        est_sigma.data[i+j] = sigma(i,j);
      }
    }
    est_sigma_pub.publish(est_sigma);
  }

  void run(){
    while(ros::ok()){
      calc();
      ros::spinOnce();
    }
  }

private:
  ros::NodeHandle nh;

  ros::Subscriber old_est_sigma_sub;
  ros::Subscriber old_est_pose_sub;
  ros::Subscriber cmd_vel_sub;

  ros::Publisher est_pose_pub;
  ros::Publisher est_sigma_pub;

  MatrixXd old_sigma;

  geometry_msgs::Pose2D est_pose;

  std_msgs::Float64MultiArray est_sigma;

  Vector3f old_est_pose;
  Vector4f alpha;

  double linear;
  double angular;

  ros::Time now;
  ros::Time old;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_predict_pose");
    PredictPose predict_pose;
    predict_pose.run();

    return 0;
}
