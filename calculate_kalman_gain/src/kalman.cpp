#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <sstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <iostream>
#include <math.h>
#include <messagefilters/subscrober.h>
#include <messagefilters/sync_policies/approximate_time.h>

using namespace Eigen;

void kalman (const std_msgs::Float64MultiArray& Sigma_t, const std_msgs::Float64MultiArray& mu_t, const std_msgs::Float64MultiArray& m_t)
{
    //行列とベクトルの宣言
    MatrixXf Sigma(3,2);
    MatrixXf H = MatrixXf::Zero(3, 3);
    MatrixXf Q = MatrixXf::Identity(3, 3);
    MatrixXf S = MatrixXf::Zero(3, 3);
    MatrixXf K = MatrixXf::Zero(3, 3);
    Vector3f mu(0, 0, 0);
    Vector3f m(0, 0, 0);
    Vector3f q(0, 0, 0);

    for(int i = 0; i < 3; ++i)
    {
        mu(i) = *mu_t[i];
        m(i) = *m_t[i];
        for(int j = 0; j < 3; ++j)
        {
            //Σへの代入
            Sigma(i,j) = *Sigma_t[i][j];
        }
    }

    q = (m(0) - mu(0)) * (m(0) - mu(0)) + (m(1)-mu(1))*(m(1)-mu(1));
   // z_h << sqrt(q), atan2(m(1)-mu(1),m(0)-mu(0))-mu(2), m(2);
    //Hの計算
    H <<
    -(mx(0)-mu(0))/sqrt(q), -(m(1)-mu(1))/sqrt(q), 0,
    m(1)-mu(1)/q,           -(m(0)-mu(0))/q,      -1,
    0,                      0,                     0;

    S = H * Sigma * H.transpose() + Q;
    K = Sigma * H.transpose() * S.inverse();

    K_pub.publish (K);
    H_pub.publish (H);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman");
    ros::NodeHandle n;

    message_filters::Subscriber<std_msgs::Float64MultiArray> Sigma_t (n, "/Sigma_t", 10);
    message_filters::Subscriber<std_msgs::Float64MultiArray> mu_t (n, "/mu_t", 10);
    message_filters::Subscriber<std_msgs::Float64MultiArray> m_t (n, "/m_t", 10);
    sync_.registerCallback(boost::bind(&kalman, this, _1, _2, _3));

    ros::Publisher K_pub = n.advertise<std_msgs::Float64MultiArray>("/K", 1000);
    ros::Publisher H_pub = n.advertise<std_msgs::Float64MultiArray>("/H", 1000);

    ros::Rate loop_rate(10);

    ros::spin();
}
