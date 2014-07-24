#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "octomap_msgs/Octomap.h"
#include "nav_msgs/Odometry.h"
#include "MSP3D.h"
#include <octomap_msgs/conversions.h>
#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/PointCloud.h"

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
octomap::point3d pstart(0,0,0);
octomap::point3d waypoint2(0,0,robot_height);
bool running=false;
bool planned=false;
std::deque<octomap::point3d> cpath;
std::vector<int> previous_ids;

double unknownSpaceProba=0.6;


void sendStop(){
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
	for(int i=0;i<5;++i){
		geometry_msgs::Point p;
		p.x=start.x();
		p.y=start.y();
		p.z=2*robot_height;
		traj_visu.points.push_back(p);
	}
	rviz_traj_pub.publish(traj_visu);
}

void measurementCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
//	if(!running)
		start=octomap::point3d(msg->pose.pose.position.x,msg->pose.pose.position.y,robot_height);
}

void goalCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	//if(!running)
		goal=octomap::point3d(msg->x,msg->y,robot_height);
		sendStop();
		planned=false;
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

void plan(){

	planned=true;
}

void octomapCallback2(const octomap_msgs::Octomap& msg)
{
	if(!planned){
		sendStop();
		plan();
	}
}

void pointcloudCallback(const sensor_msgs::PointCloud::ConstPtr& pc)
{
	bool test=false;
	for(std::vector<geometry_msgs::Point32>::const_iterator it=pc->points.begin(),end=pc->points.end();it!=end;it++){
		if((it->x*it->x+it->y*it->y+it->z*it->z)<0.5*0.5){
			test=true;
			break;
		}
	}
	//if danger
	if(test){
		sendStop();
		planned=false;
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


	//change to mean instead of max and set propability to 1 outside the allowed plane
	transformTreeToMeanVal(tree->getRoot());
	for(octomap::OcTree::tree_iterator it = tree->begin_tree(),	end=tree->end_tree(); it!= end; ++it)
	{
		if(it.getCoordinate().z()-it.getSize()/2<robot_height && it.getCoordinate().z()+it.getSize()/2>robot_height){
			if(it->getOccupancy()>unknownSpaceProba+0.01){
				it->setLogOdds(octomap::logodds(1.0));
			}else{
				if(it->getOccupancy()<unknownSpaceProba-0.01){
					it->setLogOdds(octomap::logodds(0.0));
				}else{
					it->setLogOdds(octomap::logodds(0.5));
				}
			}
		}else{
			it->setLogOdds(octomap::logodds(1.0));
		}
	}
	tree->prune();
	transformTreeToMeanVal(tree->getRoot());	

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
	transformTreeToMeanVal(tree_padded->getRoot());
	for(octomap::OcTree::tree_iterator it = tree_padded->begin_tree(),	end=tree_padded->end_tree(); it!= end; ++it)
	{
		if(it.getCoordinate().z()-it.getSize()/2<robot_height && it.getCoordinate().z()+it.getSize()/2>robot_height){
				if(it->getOccupancy()>unknownSpaceProba+0.01){
					it->setLogOdds(octomap::logodds(1.0));
				}else{
					if(it->getOccupancy()<unknownSpaceProba-0.01){
						it->setLogOdds(octomap::logodds(0.0));
					}else{
						it->setLogOdds(octomap::logodds(0.5));
					}
				}
		}else{
			it->setLogOdds(octomap::logodds(1.0));
		}
	}
	tree_padded->prune();
	transformTreeToMeanVal(tree_padded->getRoot());


	if(planned){

		//Padding
		octomap::point3d p1(-50,-50,robot_height);
		octomap::point3d p2(50,50,robot_height);
		double padding_radius=0.4;
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

		msp::MSP3D algo(*tree_padded,16);
		// "smoothing"
		octomap::point3d cur=start;
//		cpath.push_front(start);
//		int i=1;
//		while(i<cpath.size()){
//			double dinc=0.2;
//			octomap::point3d cur2=cpath[i];
//			octomap::point3d inc=(cur2-cur)*(dinc/(cur2-cur).norm());
//			int jmax= (int)floor((cur2-cur).norm()/dinc);
//			bool safe=true;
//			for(int j=1;j<jmax;++j){
//				octomap::OcTreeKey key;
//				octomap::point3d coord;
//				octomap::point3d pointToTest=cur+inc*j;
//				algo.findLRNode(pointToTest, key, coord);
//				if(tree_padded->search(key)->getOccupancy()>0.95){
//					safe=false;
//					break;
//				}
//			}
//			if(safe){
//				cpath.erase(cpath.begin()+i-1);
//			}else{
//				cur=cpath[i-1];
//				i=i+1;
//			}
//		}

		cpath.erase(cpath.begin());
		cpath.push_front(start);
		bool safe=true;
		for(int i=0;i<cpath.size()-1;i++){
			double dinc=0.1;
			cur=cpath[i];
			octomap::point3d cur2=cpath[i+1];
			octomap::point3d inc=(cur2-cur)*(dinc/(cur2-cur).norm());
			int jmax= (int)floor((cur2-cur).norm()/dinc);
			for(int j=1;j<jmax;++j){
				octomap::OcTreeKey key;
				octomap::point3d coord;
				octomap::point3d pointToTest=cur+inc*j;
				algo.findLRNode(pointToTest, key, coord);
				if(tree_padded->search(key)->getOccupancy()>0.51){
					std::cout << "obstacle on path at " << pointToTest << "in tree at "<< coord << "with value " << tree_padded->search(key)->getOccupancy()<< std::endl;
					if((pointToTest-start).norm()<10){
						safe=false;
						break;
					}
				}
			}
			if(!safe){
				break;
			}
		}
//		std::cout<< "cpath" <<std::endl;
//		for(int i=0;i<cpath.size();i++){
//			std::cout << cpath[i] << " -> ";
//		}
//		std::cout<<std::endl;
		if(!safe){
			sendStop();
			planned=false;
		}else{
//			if (cpath.front()==start){
//				cpath.erase(cpath.begin());
//			}
		}
	}

	//Padding
	octomap::point3d p1(-50,-50,robot_height);
	octomap::point3d p2(50,50,robot_height);
	double padding_radius=1.0;
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
	p11=octomap::point3d(-50,-50,robot_height);
	p22=octomap::point3d(50,50,robot_height);
	for(octomap::OcTree::leaf_bbx_iterator it2 = tree->begin_leafs_bbx (p11,p22),	end2=tree->end_leafs_bbx (); it2!= end2; ++it2)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/odom";
		marker.header.stamp = ros::Time();
		marker.ns = "tree";
		marker.id = count;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = it2.getX();
		marker.pose.position.y = it2.getY();
		marker.pose.position.z = 0;
		marker.scale.x = 0.9*it2.getSize();
		marker.scale.y = 0.9*it2.getSize();
		marker.scale.z = 0.5;
		marker.color.a = 1.0;//m_cost[i]/pow(m_nodes[i].second/m_tree.getNodeSize(m_max_tree_depth),3);
		marker.color.r = it2->getOccupancy();
		marker.color.g = it2->getOccupancy();
		marker.color.b= it2->getOccupancy();
		msg1.markers.push_back(marker);
		count++;
	}
	tree_rviz_pub.publish(msg1);


	msg1=visualization_msgs::MarkerArray();
	if(previous_ids.size()>0){
//		msg1=visualization_msgs::MarkerArray();
		for(int i=0;i<previous_ids.size();++i){
			visualization_msgs::Marker marker;
			marker.header.frame_id = "/odom";
			marker.header.stamp = ros::Time();
			marker.ns = "tree_padded";
			marker.id = previous_ids[i];
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::DELETE;
			msg1.markers.push_back(marker);
		}
//		tree_padded_rviz_pub.publish(msg1);
	}
	previous_ids.clear();


//	msg1=visualization_msgs::MarkerArray();
	count=1;
	p11=octomap::point3d(-50,-50,robot_height);
	p22=octomap::point3d(50,50,robot_height);
	for(octomap::OcTree::leaf_bbx_iterator it2 = tree_padded->begin_leafs_bbx (p11,p22),	end2=tree_padded->end_leafs_bbx (); it2!= end2; ++it2)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/odom";
		marker.header.stamp = ros::Time();
		marker.ns = "tree_padded";
		marker.id = (int)floor(10*(it2.getX()+10000*it2.getY()));
		previous_ids.push_back((int)floor(10*(it2.getX()+10000*it2.getY())));
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = it2.getX();
		marker.pose.position.y = it2.getY();
		marker.pose.position.z = 0.0;
		marker.scale.x = 0.9*it2.getSize();
		marker.scale.y = 0.9*it2.getSize();
		marker.scale.z = 0.5;
		marker.color.a = 1.0;//m_cost[i]/pow(m_nodes[i].second/m_tree.getNodeSize(m_max_tree_depth),3);
		marker.color.r = it2->getOccupancy();
		marker.color.g = it2->getOccupancy();
		marker.color.b= it2->getOccupancy();
		msg1.markers.push_back(marker);
		count++;
	}
	tree_padded_rviz_pub.publish(msg1);


	if(!planned){

	//create algo and solve
	msp::MSP3D algo(*tree_padded,16);
	algo.setGiPublisher(rviz_marker_array_pub);
	algo.setPathPublisher(rviz_traj2_pub);
	algo.init(start,goal);
	algo.setGuess(cpath);
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
	cpath=sol;
	geometry_msgs::Point waypoint;
	waypoint.x=sol[1].x();
	waypoint.y=sol[1].y();
	waypoint.z=robot_height;
	waypoint_pub.publish(waypoint);
	waypoint2=sol[1];

	//free memory
	tree->clear();
	tree_padded->clear();
	free(tree);
	free(tree_padded);
	running=false;
	planned=true;
	}
	
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
//	if (cpath.front()==start){
//		cpath.erase(cpath.begin());
//	}
//
	if(pstart==start){
		while((cpath.front()-start).norm()<0.5){
			cpath.erase(cpath.begin());
		}
	}else{
		while(((cpath.front()-start-((start-pstart)*((cpath.front()-start).dot(start-pstart)/((start-pstart).norm())))).norm()<1.0  && (cpath.front()-start).dot(start-pstart)<0)|| (cpath.front()-start).norm()<0.5){
			cpath.erase(cpath.begin());
		}
	}
	pstart=start;
	cpath.push_front(start);
	for(int i=0;i<cpath.size();++i){
		geometry_msgs::Point p;
		p.x=cpath[i].x();
		p.y=cpath[i].y();
		p.z=2*robot_height;
		traj_visu.points.push_back(p);
	}
	rviz_traj_pub.publish(traj_visu);
//	ros::Duration(5.0).sleep();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner");
  ros::NodeHandle n;
  ros::Subscriber subWaypoint = n.subscribe("octomap_full", 1, octomapCallback);
  ros::Subscriber subMeasurement = n.subscribe("odom", 1000, measurementCallback);
  ros::Subscriber subGoal = n.subscribe("planner_goal", 1000, goalCallback);
//  ros::Subscriber subpc = n.subscribe("base_scan", 1, pointcloudCallback);
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
