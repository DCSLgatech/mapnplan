#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "octomap_msgs/Octomap.h"
#include "nav_msgs/Odometry.h"
#include "MSP3D.h"
#include <octomap_msgs/conversions.h>
#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>

ros::Publisher waypoint_pub;
ros::Publisher rviz_traj_pub;
ros::Publisher rviz_traj2_pub;
ros::Publisher tree_rviz_pub;
ros::Publisher tree_padded_rviz_pub;
ros::Publisher tree_pub;
ros::Publisher rviz_marker_array_pub;
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
	transformTreeToMeanVal(tree->getRoot());
	transformTreeToMeanVal(tree_padded->getRoot());
	for(octomap::OcTree::tree_iterator it = tree->begin_tree(),	end=tree->end_tree(); it!= end; ++it)
	{
		if(it.getCoordinate().z()-it.getSize()/2<robot_height && it.getCoordinate().z()+it.getSize()/2>robot_height){
			if(it->getOccupancy()>0.51){
				it->setLogOdds(octomap::logodds(1.0));
			}else{
				if(it->getOccupancy()<0.49){
					it->setLogOdds(octomap::logodds(0.0));
				}
			}
		}else{
			it->setLogOdds(octomap::logodds(1.0));
		}
	}
	tree->prune();
	transformTreeToMeanVal(tree->getRoot());	
	for(octomap::OcTree::tree_iterator it = tree_padded->begin_tree(),	end=tree_padded->end_tree(); it!= end; ++it)
	{
		if(it.getCoordinate().z()-it.getSize()/2<robot_height && it.getCoordinate().z()+it.getSize()/2>robot_height){
				if(it->getOccupancy()>0.51){
					it->setLogOdds(octomap::logodds(1.0));
				}else{
					if(it->getOccupancy()<0.49){
						it->setLogOdds(octomap::logodds(0.0));
					}
				}
		}else{
			it->setLogOdds(octomap::logodds(1.0));
		}
	}
	tree_padded->prune();
	transformTreeToMeanVal(tree_padded->getRoot());
	//Padding
//	octomap::point3d p1(-10000,-10000,0.2);
//	octomap::point3d p2(10000,10000,0.8);
	octomap::point3d p1(-20,-20,robot_height);
	octomap::point3d p2(20,20,robot_height);
	double padding_radius=0.5;
	octomap::point3d radius(padding_radius,padding_radius,padding_radius);
	for(octomap::OcTree::leaf_bbx_iterator it = tree_padded->begin_leafs_bbx (p1,p2),	end=tree_padded->end_leafs_bbx (); it!= end; ++it)
	{
		double max=it->getOccupancy();
		octomap::point3d p11=it.getCoordinate()-radius;
		octomap::point3d p22=it.getCoordinate()+radius;
		p11.z()=robot_height;
		p22.z()=robot_height;

//		std::cout<< "limits :" << p11 << p22 << it.getCoordinate() << std::endl;
//		if(p11.z()<0.5)
//			p11.z()=0.5;
//		if(p22.z()>0.6)
//			p22.z()=0.6;
		for(octomap::OcTree::leaf_bbx_iterator it2 = tree->begin_leafs_bbx (p11,p22),	end2=tree->end_leafs_bbx (); it2!= end2; ++it2)
		{
			if(it2.getCoordinate().z()-it2.getSize()/2<robot_height && it2.getCoordinate().z()+it2.getSize()/2>robot_height){
				if(it2->getOccupancy()>max){
//					std::cout<< "updating max" << std::endl;
					max=it2->getOccupancy();
				}
			}
		}
		it->setLogOdds(octomap::logodds(max));
	}
	tree_padded->prune();
	transformTreeToMeanVal(tree_padded->getRoot());

	octomap_msgs::Octomap tree_msg;
	if(octomap_msgs::fullMapToMsg(*tree_padded,tree_msg)){
		tree_msg.header=msg.header;
		tree_pub.publish(tree_msg);
	}

	visualization_msgs::MarkerArray msg1;
	int count=1;
	octomap::point3d p11;
	octomap::point3d p22;
	p11=octomap::point3d(-20,-20,robot_height);
	p22=octomap::point3d(20,20,robot_height);
	for(octomap::OcTree::leaf_bbx_iterator it2 = tree->begin_leafs_bbx (p11,p22),	end2=tree->end_leafs_bbx (); it2!= end2; ++it2)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/odom";
		marker.header.stamp = ros::Time();
		marker.ns = "gi";
		marker.id = count;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = it2.getX();
		marker.pose.position.y = it2.getY();
		marker.pose.position.z = it2.getZ();
		marker.scale.x = 0.9*it2.getSize();
		marker.scale.y = 0.9*it2.getSize();
		marker.scale.z = 0.9*it2.getSize();
		marker.color.a = 1.0;//m_cost[i]/pow(m_nodes[i].second/m_tree.getNodeSize(m_max_tree_depth),3);
		marker.color.r = it2->getOccupancy();
		marker.color.g = it2->getOccupancy();
		marker.color.b= it2->getOccupancy();
		msg1.markers.push_back(marker);
		count++;
	}
	tree_rviz_pub.publish(msg1);


	msg1=visualization_msgs::MarkerArray();
	count=1;
	p11=octomap::point3d(-20,-20,robot_height);
	p22=octomap::point3d(20,20,robot_height);
	for(octomap::OcTree::leaf_bbx_iterator it2 = tree_padded->begin_leafs_bbx (p11,p22),	end2=tree_padded->end_leafs_bbx (); it2!= end2; ++it2)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/odom";
		marker.header.stamp = ros::Time();
		marker.ns = "gi";
		marker.id = count;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = it2.getX();
		marker.pose.position.y = it2.getY();
		marker.pose.position.z = it2.getZ();
		marker.scale.x = 0.9*it2.getSize();
		marker.scale.y = 0.9*it2.getSize();
		marker.scale.z = 0.9*it2.getSize();
		marker.color.a = 1.0;//m_cost[i]/pow(m_nodes[i].second/m_tree.getNodeSize(m_max_tree_depth),3);
		marker.color.r = it2->getOccupancy();
		marker.color.g = it2->getOccupancy();
		marker.color.b= it2->getOccupancy();
		msg1.markers.push_back(marker);
		count++;
	}
	tree_padded_rviz_pub.publish(msg1);

	//create algo and solve
	msp::MSP3D algo(*tree_padded,16);
	algo.setGiPublisher(rviz_marker_array_pub);
	algo.setPathPublisher(rviz_traj2_pub);
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
	traj_visu.scale.z=0.05;
	traj_visu.color.r = 1.0;
	traj_visu.color.a = 1.0;
	for(int i=0;i<sol.size();++i){
		geometry_msgs::Point p;
		p.x=sol[i].x();
		p.y=sol[i].y();
		p.z=2*robot_height;
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
  rviz_traj2_pub = n.advertise<visualization_msgs::Marker>("rviz_traj2", 1000);
  rviz_marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array_gi", 1000);
  tree_rviz_pub= n.advertise<visualization_msgs::MarkerArray>("marker_tree", 1000);;
  tree_padded_rviz_pub= n.advertise<visualization_msgs::MarkerArray>("marker_tree_padded", 1000);;
  tree_pub = n.advertise<octomap_msgs::Octomap>("padded_tree", 1);
  ros::spin();
  return 0;
}
