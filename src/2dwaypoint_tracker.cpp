#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

ros::Publisher command_pub;
double x_desired=0;
double y_desired=0;

double V_max=1;
double omega_max=1;
double dtheta_max=0.2;
double ds_max=0.5;
visualization_msgs::Marker traj;
double lastX=0;
double lastY=0;

void wayPointsCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  traj=*msg;
	double minds=1000;
	int ind=0;
	for(int i=0;i<traj.points.size();++i){
		double dx=lastX-traj.points[i].x;
		double dy=lastY-traj.points[i].y;
		double ds=sqrt(dx*dx+dy*dy);
		if(ds<minds){
			minds=ds;
			ind=i;
		}
	}
	traj.points.erase(traj.points.begin(),traj.points.begin()+ind);
	geometry_msgs::Point p=traj.points.front();
	traj.points.erase(traj.points.begin());
  x_desired=p.x;
	y_desired=p.y;	
}

void wayPointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  //x_desired=msg->x;
	//y_desired=msg->y;
}

double sat(double val, double lim){
	if(val>lim){
		return 1;
	}else{
		if(val<-lim){
			return -1;
		}else{
			return val/lim;
		}
	}
}

void measurementCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	lastX=msg->pose.pose.position.x;
	lastY=msg->pose.pose.position.y;
  double dx=lastX-x_desired;
  double dy=lastY-y_desired;
	double ds=sqrt(dx*dx+dy*dy);
	if(ds<0.5 && traj.points.size()>0){
		geometry_msgs::Point p=traj.points.front();
		traj.points.erase(traj.points.begin());
		x_desired=p.x;
		y_desired=p.y;
		dx=msg->pose.pose.position.x-x_desired;
		dy=msg->pose.pose.position.y-y_desired;
		ds=sqrt(dx*dx+dy*dy);
	}
	double theta_desired=atan2(dy,dx)+3.14159/2.0;
	double dtheta=tf::getYaw(msg->pose.pose.orientation)-theta_desired;
	//Put dtheta in plus or minus PI
	if(dtheta>3.14159)
		dtheta=dtheta-2*3.14159;
	if(dtheta<-3.14159)
		dtheta=dtheta+2*3.14159;
	
	double omega=omega_max*sat(-dtheta,dtheta_max);
  double V= V_max*sat(ds,ds_max);

	std::cout<<"dx: "<<dx<<", dy:"<<dy<<", dtheta: "<< dtheta << std::endl; 

	if(fabs(dtheta)>0.5){
		V=0;
	}
	if(fabs(ds)<0.01){
		V=0;
		omega=0;
	}
	geometry_msgs::Twist cmd;
	cmd.linear.x=-V;
	cmd.angular.z=omega;
	command_pub.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twoDWayPointTracker");
  ros::NodeHandle n;
  ros::Subscriber subWaypoint = n.subscribe("waypoint", 1000, wayPointCallback);
  ros::Subscriber subWaypoints = n.subscribe("rviz_traj", 1000, wayPointsCallback);
  ros::Subscriber subMeasurement = n.subscribe("odom", 1000, measurementCallback);
	command_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::spin();
  return 0;
}
