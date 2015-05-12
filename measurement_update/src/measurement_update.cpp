#include "ros/ros.h"

#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"

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

class MeasurementUpdate{
public:
  MeasurementUpdate(){
		z_t_sub = nh.subscribe("z_t",10,&MeasurementUpdate::z_tCallBack, this);
    mu_t_bar_sub = nh.subscribe("mu_t_bar",10, &MeasurementUpdate::mu_t_barCallBack, this);
		sigma_t_bar_sub = nh.subscribe("sigma_t_bar",10, &MeasurementUpdate::sigma_t_barCallBack, this);
		k_sub = nh.subscribe("k",10,&MeasurementUpdate::kCallBack, this);
		h_sub = nh.subscribe("h",10,&MeasurementUpdate::hCallBack, this);

		mu_pub = nh.advertise<std_msgs::Float64MultiArray>("/mu",1000);
		sigma_pub = nh.advertise<std_msgs::Float64MultiArray>("sigma",1000);

		k = MatrixXf::Zero(3, 3);
		h = MatrixXf::Zero(3, 3);
		sigma_bar = MatrixXf::Zero(3, 3);

    ros::Rate loop_rate(10);
  }

  void z_tCallBack(const geometry_msgs::Pose2D::ConstPtr& z_t){
		z[0] = z_t->x;
		z[1] = z_t->y;
		z[2] = z_t->theta;
  }

  void mu_t_barCallBack(const geometry_msgs::Pose2D::ConstPtr& mu_t_bar){
		mu_bar[0] = mu_t_bar->x;
		mu_bar[1] = mu_t_bar->y;
		mu_bar[2] = mu_t_bar->theta;
  }

  void sigma_t_barCallBack(const std_msgs::Float64MultiArray::ConstPtr& sigma_t_bar){
    for(int i=0; i<3; i++){
      for(int j=0; j<3; j++){
        sigma_bar(i,j) = sigma_t_bar->data[i+j];
      }
    }
  }

	void kCallBack(const std_msgs::Float64MultiArray::ConstPtr& k_t){
		for (int i=0; i< 3; i++){
			for (int j=0; j<3; j++){
				k(i) = k_t->data[i];
			}
		}
	}

	void hCallBack(const std_msgs::Float64MultiArray::ConstPtr& h_t){
		for (int i=0; i<3; i++){
			for (int j=0; j<3; i++){
				h(i) = h_t->data[i];
			}
		}
	}

  void calc(){
		Vector3f mu = Vector3f::Zero();
		MatrixXf sigma = MatrixXf::Zero(3, 3);
		MatrixXf I = MatrixXf::Zero(3, 3);

		geometry_msgs::Pose2D mu_p;
		std_msgs::Float64MultiArray sigma_p;

		sigma_p.data.resize(9);

		int i, j, l;

		I <<
			1, 0, 0,
			0, 1, 0,
			0, 0, 1;

		 mu = mu_bar + k *  (z - mu_bar);

		sigma = (I - k * h) * sigma_bar;

		/* mu_p.x = mu(0);
		mu_p.y = mu(1);
		mu_p.theta = mu(2);

		for(i=0; i<3; i++){
			for(j=0;j<3;j++){
				sigma_p.data[i+j] = sigma(i,j);
			}
		}

		mu_pub.publish (mu_p);
		sigma_pub.publish (sigma_p); */
  }


  void run(){
    while(ros::ok()){
      calc();
      ros::spinOnce();
    }
  }

private:
	ros::Subscriber z_t_sub;
	ros::Subscriber mu_t_bar_sub;
	ros::Subscriber sigma_t_bar_sub;
	ros::Subscriber k_sub;
	ros::Subscriber h_sub;

	ros::Publisher mu_pub;
	ros::Publisher sigma_pub;

  ros::NodeHandle nh;

	Vector3f z;
	Vector3f mu_bar;
	MatrixXf sigma_bar;
	MatrixXf k;
	MatrixXf h;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "MeasurementUpdate");
    MeasurementUpdate mesurement_update;
    mesurement_update.run();

    return 0;
}
