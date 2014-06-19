#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "octomap_msgs/Octomap.h"
#include "nav_msgs/Odometry.h"
#include "MSP3D.h"
#include <octomap_msgs/conversions.h>
#include "visualization_msgs/Marker.h"

ros::Publisher waypoint_pub;
ros::Publisher rviz_traj_pub;
octomap::point3d goal(0,0,0);
octomap::point3d start(0,0,0);
octomap::point3d waypoint2(0,0,0.545);
bool running=false;

double unknownSpaceProba=0.5;


void measurementCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(!running)
		start=octomap::point3d(msg->pose.pose.position.x,msg->pose.pose.position.y,0.545);
}

void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	if(!running)
		goal=octomap::point3d(msg->x,msg->y,0.545);
}

void transformTreeToMeanVal(octomap::OcTreeNode* node){
	if(node->hasChildren()){
		for (unsigned int i=0; i<8; i++) {
	    	if (node->childExists(i)) {
	      		transformTreeToMeanVal(node->getChild(i));
	    	}else{
				node->createChild(i);
				node->getChild(i)->setLogOdds(octomap::logodds(unknownSpaceProba));
			}
	  }
		node->setLogOdds(node->getMeanChildLogOdds());
	}
}

void octomapCallback(const octomap_msgs::Octomap& msg)
{
	running=true;
	/*if(waypoint2.distanceXY(start)>0.1){
		if(waypoint2==octomap::point3d(0,0,0.545)){
			waypoint2=start;
		}
		std::cout<< "distance to waypoint:" << (waypoint2-start).norm() << std::endl;
		geometry_msgs::Point waypoint;
		waypoint.x=waypoint2.x();
		waypoint.y=waypoint2.y();
		waypoint.z=waypoint2.z();
		waypoint_pub.publish(waypoint);
		running=false;
		return;
	}*/
	//load tree
	octomap::OcTree* tree;
	octomap::AbstractOcTree* tree1=octomap_msgs::msgToMap(msg);
	if(tree){ // read error returns NULL
		tree = dynamic_cast<octomap::OcTree*>(tree1);
		if (tree){ // cast succeeds if correct type
		// do something....
		}else{
			return;
		}
	}else{
		return;
	}

	//change to mean instead of max and set propability to 1 outside the allowed plane
	// transformTreeToMeanVal(tree->getRoot());	

	octomap::point3d m_try(10.0,-2.0,0.5);
	// ROS_INFO("#######################################");
	// ROS_INFO("planner.cpp:\t%f",tree->search(tree->coordToKey(m_try))->getOccupancy());
	// ROS_INFO("#######################################");

	for(octomap::OcTree::tree_iterator it = tree->begin_tree(),	end=tree->end_tree(); it!= end; ++it)
	{
		if(it.getCoordinate().z()<0.8 && it.getCoordinate().z()>0.2)
			int bananer = 0;
		else
			it->setLogOdds(octomap::logodds(1.0));
	}

	// ROS_INFO("#######################################");
	// ROS_INFO("planner.cpp 2:\t%f",tree->search(tree->coordToKey(m_try))->getOccupancy());
	// ROS_INFO("#######################################");

	transformTreeToMeanVal(tree->getRoot());	

	ROS_INFO("#######################################");
	ROS_INFO("planner.cpp 3:\t%f",tree->search(tree->coordToKey(m_try))->getOccupancy());
	ROS_INFO("#######################################");

	//create algo and solve	
	msp::MSP3D algo(*tree,16);	
	algo.init(start,goal);
	algo.run();

	//get solution and publish
	std::deque<octomap::point3d> sol=algo.getPath();
	if(sol.size()<2){
		std::cout<< "invalid solution" << std::endl;
		running=false;
		return;
	}
	geometry_msgs::Point waypoint;
	waypoint.x=sol[1].x();
	waypoint.y=sol[1].y();
	waypoint.z=0.545;
	waypoint_pub.publish(waypoint);
	waypoint2=sol[1];
	
	//publish traj
	visualization_msgs::Marker traj_visu;
	traj_visu.header.frame_id="/odom";
	traj_visu.header.stamp=ros::Time::now();
	traj_visu.ns="traj";
	traj_visu.type=visualization_msgs::Marker::LINE_STRIP;
	traj_visu.action=visualization_msgs::Marker::ADD;
	traj_visu.id=1;
	traj_visu.scale.x=0.05;
	traj_visu.scale.y=0.05;
	traj_visu.color.r = 1.0;
	traj_visu.color.a = 1.0;
	for(int i=0;i<sol.size();++i){
		geometry_msgs::Point p;
		p.x=sol[i].x();
		p.y=sol[i].y();
		p.z=sol[i].z();
		traj_visu.points.push_back(p);
	}
	rviz_traj_pub.publish(traj_visu);

	//free memory
	free(tree);
	running=false;
	ros::Duration(10.0).sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  ros::Subscriber subWaypoint = n.subscribe("octomap_full", 1, octomapCallback);
  ros::Subscriber subMeasurement = n.subscribe("odom", 1000, measurementCallback);
  ros::Subscriber subGoal = n.subscribe("planner_goal", 1000, goalCallback);
  waypoint_pub = n.advertise<geometry_msgs::Point>("waypoint", 1000);
  rviz_traj_pub = n.advertise<visualization_msgs::Marker>("rviz_traj", 1000);
  ros::spin();
  return 0;
}
