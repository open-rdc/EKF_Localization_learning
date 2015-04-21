#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <iostream>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <complex>

using namespace Eigen;

class Kalman{
public:
  Kalman(){
    Sigma_t_sub = nh.subscribe("/Sigma_t",10, &Kalman::sigma_tCallBack, this);
    mu_t_sub = nh.subscribe("/mu_t",10, &Kalman::mu_tCallBack, this);
    m_t_sub = nh.subscribe("/m_t",10, &Kalman::m_tCallBack, this);

    K_pub = nh.advertise<std_msgs::Float64MultiArray>("/K", 1000);
    H_pub = nh.advertise<std_msgs::Float64MultiArray>("/H", 1000);

    ros::Rate loop_rate(10);
  }

  void sigma_tCallBack(const std_msgs::Float64MultiArray::ConstPtr& Sigma_t){
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
        Sigma(i,j) = Sigma_t->data[i+j];
      }
    }
  }

  void mu_tCallBack(const std_msgs::Float64MultiArray::ConstPtr& mu_t){
    for(int i=0; i<3; i++){
      mu(i) = mu_t->data[i];
    }
  }

  void m_tCallBack(const std_msgs::Float64MultiArray::ConstPtr& m_t){
    for(int i=0; i<3; i++){
      m(i) = m_t->data[i];
    }
  }

  void calc(){
    MatrixXf H = MatrixXf::Zero(3, 3);
    MatrixXf Q = MatrixXf::Identity(3, 3);
    MatrixXf S = MatrixXf::Zero(3, 3);
    MatrixXf K = MatrixXf::Zero(3, 3);
    Sigma = MatrixXf::Zero(3,3);
    mu = Vector3f::Zero();
    mu = Vector3f::Zero();

    float q=0;
    int i,j;
    std_msgs::Float64MultiArray K_p;
    std_msgs::Float64MultiArray H_p;

    q = (m(0)-mu(0))*(m(0)-mu(0)) + (m(1)-mu(1))*(m(1)-mu(1));
    // z_h << sqrt(q), atan2(m(1)-mu(1),m(0)-mu(0))-mu(2), m(2);
    //Hの計算
    H <<
    -(m(0)-mu(0))/sqrt(q), -(m(1)-mu(1))/sqrt(q), 0,
    m(1)-mu(1)/q,           -(m(0)-mu(0))/q,      -1,
    0,                      0,                     0;

    S = H * Sigma * H.transpose() + Q;
    K = Sigma * H.transpose() * S.inverse();

    for(i = 0; i<3; ++i)
    {
        for(j =0; j<3; ++j)
        {
            K_p.data[i+j] = K(i,j);
            H_p.data[i+j] = H(i,j);
        }
    }

    K_pub.publish (K_p);
    H_pub.publish (H_p);
    //ToDo
  }

  void run(){
    while(ros::ok()){
      calc();
      ros::spinOnce();
    }
  }

private:
  ros::Subscriber Sigma_t_sub;
  ros::Subscriber mu_t_sub;
  ros::Subscriber m_t_sub;

  ros::Publisher K_pub;
  ros::Publisher H_pub;

  ros::NodeHandle nh;

  MatrixXf Sigma;
  Vector3f mu;
  Vector3f m;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman");
    Kalman kalman;
    kalman.run();

    return 0;
}
