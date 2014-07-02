#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "octomap_msgs/Octomap.h"
#include "nav_msgs/Odometry.h"
#include "MSP3D.h"
#include <octomap_msgs/conversions.h>
#include "visualization_msgs/Marker.h"

ros::Publisher waypoint_pub;
ros::Publisher rviz_traj_pub;
ros::Publisher tree_pub;
double robot_height=0.545;
octomap::point3d goal(0,0,0);
octomap::point3d start(0,0,0);
octomap::point3d waypoint2(0,0,robot_height);
bool running=false;

double unknownSpaceProba=0.5;


void measurementCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	if(!running)
		start=octomap::point3d(msg->pose.pose.position.x,msg->pose.pose.position.y,robot_height);
}

void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	if(!running)
		goal=octomap::point3d(msg->x,msg->y,robot_height);
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
	//load tree
	octomap::OcTree* tree;
	octomap::AbstractOcTree* tree1=octomap_msgs::msgToMap(msg);
	if(tree1){ // read error returns NULL
		tree = dynamic_cast<octomap::OcTree*>(tree1);
		if (tree){ // cast succeeds if correct type
		// do something....
		}else{
			return;
		}
	}else{
		return;
	}
	octomap::OcTree* tree_padded;
	octomap::AbstractOcTree* tree_padded1=octomap_msgs::msgToMap(msg);
	if(tree_padded1){ // read error returns NULL
		tree_padded = dynamic_cast<octomap::OcTree*>(tree_padded1);
		if (tree_padded){ // cast succeeds if correct type
		// do something....
		}else{
			return;
		}
	}else{
		return;
	}

	//change to mean instead of max and set propability to 1 outside the allowed plane
	for(octomap::OcTree::tree_iterator it = tree->begin_tree(),	end=tree->end_tree(); it!= end; ++it)
	{
		if(it.getCoordinate().z()-it.getSize()/2<robot_height && it.getCoordinate().z()+it.getSize()/2>robot_height){
			if(it->getOccupancy()>0.6){
				it->setLogOdds(octomap::logodds(1.0));
			}
		}else{
			it->setLogOdds(octomap::logodds(1.0));
		}
	}
	transformTreeToMeanVal(tree->getRoot());	
	for(octomap::OcTree::tree_iterator it = tree_padded->begin_tree(),	end=tree_padded->end_tree(); it!= end; ++it)
	{
		if(it.getCoordinate().z()-it.getSize()/2<robot_height && it.getCoordinate().z()+it.getSize()/2>robot_height){
				if(it->getOccupancy()>0.6){
					it->setLogOdds(octomap::logodds(1.0));
				}
		}else{
			it->setLogOdds(octomap::logodds(1.0));
		}
	}
	transformTreeToMeanVal(tree_padded->getRoot());
	//Padding
//	octomap::point3d p1(-10000,-10000,0.2);
//	octomap::point3d p2(10000,10000,0.8);
	octomap::point3d p1(-10000,-10000,0.0);
	octomap::point3d p2(10000,10000,1.0);
	double padding_radius=0.4;
	octomap::point3d radius(padding_radius,padding_radius,padding_radius);
	for(octomap::OcTree::leaf_bbx_iterator it = tree_padded->begin_leafs_bbx (p1,p2),	end=tree_padded->end_leafs_bbx (); it!= end; ++it)
	{
		double max=it->getLogOdds();
		octomap::point3d p11=it.getCoordinate()-radius;
		octomap::point3d p22=it.getCoordinate()+radius;
		if(p11.z()<0.5)
			p11.z()=0.5;
		if(p22.z()>0.6)
			p22.z()=0.6;
		for(octomap::OcTree::leaf_bbx_iterator it2 = tree->begin_leafs_bbx (p11,p22),	end2=tree->end_leafs_bbx (); it2!= end2; ++it2)
		{
			if(it2->getLogOdds()>max){
				max=it2->getLogOdds();
			}
		}
		it->setLogOdds(max);
	}
	transformTreeToMeanVal(tree_padded->getRoot());

	octomap_msgs::Octomap tree_msg;
	if(octomap_msgs::fullMapToMsg(*tree_padded,tree_msg)){
		tree_msg.header=msg.header;
		tree_pub.publish(tree_msg);
	}

	//create algo and solve	
	msp::MSP3D algo(*tree_padded,16);
	algo.init(start,goal);
	if (!algo.run()){
		return;
	}
	std::cout<<"Goal: "<< goal << std::endl;
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
	waypoint.z=robot_height;
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
  tree_pub = n.advertise<octomap_msgs::Octomap>("padded_tree", 1);
  ros::spin();
  return 0;
}
