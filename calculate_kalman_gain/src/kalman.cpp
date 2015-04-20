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
ros::Publisher K_pub;
ros::Publisher H_pub;

void kalman (const std_msgs::Float64MultiArray::ConstPtr& Sigma_t, const std_msgs::Float64MultiArray::ConstPtr& mu_t, const std_msgs::Float64MultiArray::ConstPtr& m_t)
{
    //行列とベクトルの宣言
    MatrixXf Sigma(3,2);
    MatrixXf H = MatrixXf::Zero(3, 3);
    MatrixXf Q = MatrixXf::Identity(3, 3);
    MatrixXf S = MatrixXf::Zero(3, 3);
    MatrixXf K = MatrixXf::Zero(3, 3);
    Vector3f mu(0, 0, 0);
    Vector3f m(0, 0, 0);
    float q=0;
    int i,j;
    std_msgs::Float64MultiArray K_p;
    std_msgs::Float64MultiArray H_p;


    for(i = 0; i < 3; ++i)
    {
        mu(i) = mu_t -> data[i];
        m(i) = m_t -> data[i];
        for(j = 0; j < 3; ++j)
        {
            //Σへの代入
            Sigma(i,j) = Sigma_t -> data[i+j];
        }
    }

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

}


int main(int argc, char **argv)
{
 //   typedef message_filters::sync_policies::ApproximateTime<std_msgs::Float64MultiArray::ConstPtr,std_msgs::Float64MultiArray::ConstPtr,std_msgs::Float64MultiArray::ConstPtr> SyncPolicy;
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;


   //message_filters::Subscriber<std_msgs::Float64MultiArray::ConstPtr> Sigma_t (n, "/Sigma_t", 10);
   //message_filters::Subscriber<std_msgs::Float64MultiArray::ConstPtr> mu_t (n, "/mu_t", 10);
   //message_filters::Subscriber<std_msgs::Float64MultiArray::ConstPtr> m_t (n, "/m_t", 10);
    
   //message_filters::Synchronizer<SyncPolicy> sync_(SyncPolicy(10),Sigma_t,mu_t,m_t);
   //sync_.registerCallback(boost::bind(&kalman, _1, _2, _3));

    K_pub = n.advertise<std_msgs::Float64MultiArray>("/K", 1000);
    H_pub = n.advertise<std_msgs::Float64MultiArray>("/H", 1000);

    ros::Rate loop_rate(10);

    ros::spin();
}
