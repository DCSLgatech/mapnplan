#include "MSP3D.h"
#include "kshortestpaths/YenTopKShortestPathsAlg.h"
#include "kshortestpaths/DijkstraShortestPathAlg.h"
#include <set>
#include <octomap/ColorOcTree.h>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <visualization_msgs/MarkerArray.h>


namespace msp{
MSP3D::MSP3D(octomap::OcTree &tree, int max_depth):m_tree(tree),m_path_found(false),m_visu(false),m_alpha(3.0),m_eps(tree.getResolution()/10.0),m_max_tree_depth(max_depth),m_lambda1(0.5),m_lambda2(0.5) {
	m_M=100*pow(8,max_depth);
	m_epsilon=0.49;
	//	m_epsilon=pow(0.5,1+3*m_max_tree_depth);
	//m_epsilon=0.0000025;
	m_child_dir.push_back(octomap::point3d(-1,-1,-1));
	m_child_dir.push_back(octomap::point3d(1,-1,-1));
	m_child_dir.push_back(octomap::point3d(-1,1,-1));
	m_child_dir.push_back(octomap::point3d(1,1,-1));
	m_child_dir.push_back(octomap::point3d(-1,-1,1));
	m_child_dir.push_back(octomap::point3d(1,-1,1));
	m_child_dir.push_back(octomap::point3d(-1,1,1));
	m_child_dir.push_back(octomap::point3d(1,1,1));
}


void MSP3D::setGiPublisher(ros::Publisher pub){
	m_pub=pub;
}

bool MSP3D::init(octomap::point3d start,octomap::point3d end){
	if(findLRNode(start, m_start, m_start_coord) && findLRNode(end, m_end, m_end_coord)){
		//		std::cout << "Start :" << start << std::endl;
		//		std::cout << m_start_coord << std::endl;
		//		std::cout << "Goal :" << end << std::endl;
		//		std::cout << m_end_coord << std::endl;
		m_current_coord=m_start_coord;
		m_path_cost.clear();
		m_full_path_estimate_cost.clear();
		m_current_path.push_back(m_start_coord);
		m_misleading.clear();
		m_misleading[m_current_coord]=std::set<octomap::point3d,Point3D_Less>();
		m_nb_step=0;
		m_nb_backtrack=0;
		m_speed_up=true;
		return true;
	}else{
		ROS_INFO("start or goal not on map");
		exit(1);
	}

	return false;
}

bool MSP3D::step(){
	//	std::cout << "Calculate Graph" << std::endl;
	reducedGraph();
	// shortest path
	kshortestpaths::YenTopKShortestPathsAlg yenAlg(m_graph, m_graph.get_vertex(m_start_index),m_graph.get_vertex(m_end_index));
	bool got_next=false;
	//	std::cout << "Solve shortest path" << std::endl;
	if(yenAlg.has_next()){
		got_next=true;
	}
	//return false;
	//	std::cout << "End of step update" << std::endl;

	//TODO: check if the path contains obstacles (cost > M) if yes change get next to false

	//if solution
	if(got_next){
		//go forward // if goal return false;
		kshortestpaths::BasePath* result =yenAlg.next();
		//		std::cout << "Cost: " << result->Weight() << " Length: " << result->length() << std::endl;
		double full_path_estimate_cost=getPathCost()+result->Weight();
		if(result->Weight()>=m_M){// || (!m_full_path_estimate_cost.empty() &&full_path_estimate_cost>m_full_path_estimate_cost.back())){
			//no path without obstacles from current to finish  or setback
			//			std::cout << "shortest path with obstacles" << std::endl;
			got_next=false;
		}else{
			//			std::cout << "shortest path found" << std::endl;
			//			for(int i=0;i<result->length();++i)
			//			{
			//				std::cout << m_nodes[result->GetVertex(i)->getID()].first;
			//				std::cout << "->";
			//			}
			//			std::cout << std::endl <<  "*********************************************" << std::endl;
			int next_point_id=result->GetVertex(1)->getID();
			//do stuff to prepare next iteration
			//m_visited[m_current_coord]=	m_visited[m_current_coord]+1;

			m_misleading[m_current_coord].insert(m_nodes[next_point_id].first);
//			m_misleading[m_start_coord].insert(m_nodes[next_point_id].first);

			//		std::cout<<"before adding element"<< std::endl;
			//		for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
			//			std::cout<< (*it) << std::endl;
			//		}

			m_current_path.push_back(m_nodes[next_point_id].first);
			m_path_cost.push_back(m_cost[next_point_id]);
			m_full_path_estimate_cost.push_back(full_path_estimate_cost);

			if(m_speed_up){
				int mv_fwd=2;
				while(result->length()>mv_fwd){
					int next_point_id2=result->GetVertex(mv_fwd)->getID();
					//TODO : repalce test by if next vertex at finest resolution, aka, no children
					if(m_nodes[next_point_id2].second==m_nodes[next_point_id].second){
						//m_visited[m_nodes[next_point_id].first]=	m_visited[m_nodes[next_point_id].first]+1;
						m_misleading[m_nodes[next_point_id].first].insert(m_nodes[next_point_id2].first);
//						m_misleading[m_start_coord].insert(m_nodes[next_point_id2].first);
						m_current_path.push_back(m_nodes[next_point_id2].first);
						m_path_cost.push_back(m_cost[next_point_id2]);
						m_full_path_estimate_cost.push_back(full_path_estimate_cost);
						next_point_id=next_point_id2;
						++mv_fwd;
					}else{
						break;
					}
				}
			}

			if(getPathCost()>=m_M){
				//no path without obstacles from start to current
				return false;
			}
			//m_current_point;
			m_current_coord=m_nodes[next_point_id].first;

			if(next_point_id==m_end_index){
				std::cout << "goal reached" << std::endl;
				std::cout << "goal" <<m_nodes[m_end_index].first << ", "<<  m_nodes[m_end_index].second<< std::endl;
				m_path_found=true;
				//				std::stringstream it_name;
				//				it_name << "iteration0.ot";
				//				visu_end(std::string(it_name.str()));
				//				std::cout << it_name.str() << std::endl;
				return false;
			}else{
				visualization_msgs::Marker traj_visu;
				traj_visu.header.frame_id="/odom";
				traj_visu.header.stamp=ros::Time::now();
				traj_visu.ns="traj2";
				traj_visu.type=visualization_msgs::Marker::LINE_STRIP;
				traj_visu.action=visualization_msgs::Marker::ADD;
				traj_visu.id=1;
				traj_visu.scale.x=0.05;
				traj_visu.scale.y=0.05;
				traj_visu.scale.z=0.05;
				traj_visu.color.g = 1.0;
				traj_visu.color.a = 1.0;
				for(int i=0;i<m_current_path.size();++i){
					geometry_msgs::Point p;
					p.x=m_current_path[i].x();
					p.y=m_current_path[i].y();
					p.z=2;
					traj_visu.points.push_back(p);
				}
				for(int i=0;i<result->length();++i){
					geometry_msgs::Point p;
					p.x=m_nodes[result->GetVertex(i)->getID()].first.x();
					p.y=m_nodes[result->GetVertex(i)->getID()].first.y();
					p.z=2;
					traj_visu.points.push_back(p);
				}
				m_rviz_traj_pub.publish(traj_visu);
				return true;
			}
		}
	}
	if(!got_next){
		//std::cout << "shortest path not found" << std::endl;
		//go back // if path empty => no solution return false
		//m_visited[m_current_coord]=0;
//		m_misleading[m_current_coord].clear();
//		m_misleading[m_start_coord].clear();
		m_nb_backtrack++;
		m_current_path.pop_back();
		if(m_current_path.size()==0){
			//no possible path
			return false;
		}else{
//			m_misleading[m_current_path.back()].insert(m_current_coord);
			m_current_coord=m_current_path.back();
			m_path_cost.pop_back();
			m_full_path_estimate_cost.pop_back();
			return true;
		}
	}
}

bool MSP3D::run(){
	//	if(m_tree.search(m_start)->getOccupancy()>1-m_epsilon || m_tree.search(m_end)->getOccupancy()>1-m_epsilon){
	//		ROS_INFO("goal : %f",m_tree.search(m_end)->getOccupancy());
	//		ROS_INFO("start : %f",m_tree.search(m_start)->getOccupancy());
	//		ROS_INFO("start or end is an obstacle");
	//		return false;
	//	}
	while(step()){/*std::cout<<*/++m_nb_step;}
	std::cout<< "NB backtrack : " << m_nb_backtrack << std::endl;
	if(m_path_found){
		std::cout << "Goal MSP3D: " << m_end_coord << std::endl;
		ROS_INFO("path found");
		return true;
	}else{
		ROS_INFO("path  not found");
		return false;
	}
}


bool MSP3D::runAs(){
	clock_t tstart = clock();
	Gfull();
	clock_t tend1 = clock();
	kshortestpaths::DijkstraShortestPathAlg shortest_path_alg(&m_graph);
	kshortestpaths::BasePath* result =
			shortest_path_alg.get_shortest_path(
					m_graph.get_vertex(m_start_index), m_graph.get_vertex(m_end_index));
	clock_t tend2 = clock();
	std::cout << "Time to calculate GFull: " << (tend1-tstart)*1.0/CLOCKS_PER_SEC << std::endl;
	std::cout << "Time to calculate A*: " << (tend2-tend1)*1.0/CLOCKS_PER_SEC << std::endl;
	std::cout << "Total time: " << (tend2-tstart)*1.0/CLOCKS_PER_SEC << std::endl;
	std::cout <<  "start :" << m_start_index << ", end :" << m_end_index << std::endl;
	//	double scale=m_tree.getResolution()*pow(2,16-m_max_tree_depth)/(1-8);
	//	for(int i=0;i<result->length();++i)
	//	{
	//		//std::cout << m_nodes[result->GetVertex(i)->getID()].first;
	//		std::cout << (((m_nodes[result->GetVertex(i)->getID()].first)*(1/scale))+octomap::point3d(3.5,3.5,3.5))*(1.0/7.0);
	//		std::cout << "->";
	//	}
	//	std::cout << std::endl <<  "*********************************************" << std::endl;
	std:;cout << "Cost: " << result->Weight() << " Length: " << result->length() << std::endl;
}

std::deque<octomap::point3d> MSP3D::getPath(){
	// "smoothing"
	octomap::point3d cur=m_current_path.front();
	int i=2;
	while(i<m_current_path.size()){
		octomap::point3d cur2=m_current_path[i];
		octomap::point3d inc=(cur2-cur)*(0.05/(cur2-cur).norm());
		int jmax= (int)floor((cur2-cur).norm()/0.05);
		bool safe=true;
		for(int j=1;j<jmax;++j){
			octomap::OcTreeKey key;
			octomap::point3d coord;
			octomap::point3d pointToTest=cur+inc*j;
			findLRNode(pointToTest, key, coord);
			if(m_tree.search(key)->getOccupancy()>1-m_epsilon){
				safe=false;
				break;
			}
		}
		if(safe){
				m_current_path.erase(m_current_path.begin()+i-1);
		}else{
			cur=m_current_path[i-1];
			i=i+1;
		}
	}
	return m_current_path;
}

double MSP3D::getPathCost(){
	return std::accumulate(m_path_cost.begin(),m_path_cost.end(),0.0);//
	//	double cost=0;
	//	std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();
	//	++it;
	//	for(0;it!=end;++it){
	//		cost += low_cost(*it);
	////		std::cout << low_cost(*it) << " -> ";
	//	}
	////	std::cout<<std::endl;
	//	return cost;
}

bool MSP3D::inPath(octomap::point3d pt,double size){
	//	std::cout << "in path intro" << std::endl;
	//	for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){
	//				std::cout<< (*it) << std::endl;
	//			}


	//TODO : change it to if it includes an element of the path, i e, if is_in(*it, pt, pt_size)

	for(std::deque<octomap::point3d>::iterator it=m_current_path.begin(),end=m_current_path.end();it!=end;++it){

		//		std::cout << "in path test " <<pt << (*it) << std::endl;
		//if(pt==(*it) && !((*it)==m_current_path.back())){
		if(is_in(*it,std::pair<octomap::point3d,double>(pt,size)) && !((*it)==m_current_path.back())){
			//			std::cout << "in path " << std::endl;
			return true;
		}
	}
	return false;
}

bool MSP3D::findLRNode(octomap::point3d& pt,octomap::OcTreeKey& key, octomap::point3d& coord){
	octomap::OcTreeNode* node=m_tree.getRoot();
	octomap::point3d pos(0,0,0);
	double size=m_tree.getNodeSize(0);
	while(node->hasChildren()){
		bool updated=false;
		for(int i=0;i<8;++i){
			//			std::cout << "pt " << pt <<std::endl;
			//			std::cout << "pos " << pos+m_child_dir[i]*0.25*size <<std::endl;
			//			std::cout << "size " << 0.5*size <<std::endl;
			if(is_in(pt,std::pair<octomap::point3d,double>(pos+m_child_dir[i]*0.25*size,size*0.5))){
				node=node->getChild(i);
				pos=pos+m_child_dir[i]*0.25*size;
				size=size*0.5;
				updated=true;
				break;
			}
		}
		if(!updated){
			std::cout<<"Error in datastructure"<< std::endl;
			return false;
		}
	}
	m_tree.coordToKeyChecked(pos,key);
	coord=pos;
	return true;
}

void MSP3D::add_node_to_reduced_vertices(octomap::OcTreeNode* node,octomap::point3d coord, double size){
	//	std::cout << coord << " , " << size << " , " << node->getOccupancy() <<std::endl;
	double robot_height=0.545;
	if(((coord-m_current_coord).norm()>m_alpha*size  || !(node->hasChildren())) ){
		if(!inPath(coord,size)
				&& node->getOccupancy()<1-m_epsilon/pow(size/m_tree.getResolution(),3)
				&& m_current_forbidden.find(coord)==m_current_forbidden.end()
				&& coord.z()-size*0.5<robot_height
				&& coord.z()+size*0.5>robot_height
		){
			m_nodes.push_back(std::pair<octomap::point3d,double>(coord,size));
			m_cost.push_back(cost_func(node->getOccupancy(),size)*pow(size/m_tree.getNodeSize(m_max_tree_depth),3));
		}else{
			std::pair<octomap::point3d,double> pn(coord,size+0.5);
			if(!(node->hasChildren()) && is_in(m_start_coord,pn) && !inPath(coord,size) ){
				m_nodes.push_back(std::pair<octomap::point3d,double>(coord,size));
				m_cost.push_back(cost_func(node->getOccupancy(),size)*pow(size/m_tree.getNodeSize(m_max_tree_depth),3));
			}
		}
	}else{
		for(int i=0;i<8;++i){
//			if(size<10.0 && !node->childExists(i)){
//				node->createChild(i);
//			}
			if(node->childExists(i)){
				add_node_to_reduced_vertices(node->getChild(i),coord+m_child_dir[i]*0.25*size,size*0.5);
			}
		}
	}
}

void MSP3D::publishGiRviz(){
	visualization_msgs::MarkerArray msg;
	int count=1;
	for(int i=0;i<m_nodes.size();++i){
		if(fabs(m_nodes[i].first.x())>20 || fabs(m_nodes[i].first.y())>20){
			continue;
		}
		visualization_msgs::Marker marker;
		marker.header.frame_id = "/odom";
		marker.header.stamp = ros::Time();
		marker.ns = "gi";
		marker.id = count;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = m_nodes[i].first.x();
		marker.pose.position.y = m_nodes[i].first.y();
		marker.pose.position.z = 1;
		marker.scale.x = 0.9*m_nodes[i].second;
		marker.scale.y = 0.9*m_nodes[i].second;
		marker.scale.z = 0.5;
		marker.color.a = 1.0;//m_cost[i]/pow(m_nodes[i].second/m_tree.getNodeSize(m_max_tree_depth),3);
		marker.color.r = m_cost[i]/pow(m_nodes[i].second/m_tree.getNodeSize(m_max_tree_depth),3);
		marker.color.g = m_cost[i]/pow(m_nodes[i].second/m_tree.getNodeSize(m_max_tree_depth),3);
		marker.color.b= m_cost[i]/pow(m_nodes[i].second/m_tree.getNodeSize(m_max_tree_depth),3);
		msg.markers.push_back(marker);
		count++;
	}
	//	std::cout << "publishing gi with nb markers " << count <<  std::endl;
	m_pub.publish(msg);
	ros::Duration(1.0).sleep();
}

void MSP3D::reducedGraph(){
	m_graph.clear();
	m_nodes.clear();
	m_cost.clear();
	m_start_index=-1;
	m_end_index=-1;
	try {
		m_current_forbidden=m_misleading.at(m_current_coord);
//		m_current_forbidden=m_misleading.at(m_start_coord);
	}catch (const std::out_of_range& oor) {
		m_current_forbidden=std::set<octomap::point3d,Point3D_Less>();
	}
	add_node_to_reduced_vertices(m_tree.getRoot(),octomap::point3d(0,0,0),m_tree.getNodeSize(0));

	//publishGiRviz();

	int l=m_nodes.size();

	//	std::cout<< "number of nodes: " << l << std::endl;
	for(int i=0;i<l;++i){
		// !!!!!!!!!!!!!  if not in path?
		//		std::cout<< "node " << i << ":" << m_nodes[i].first <<std::endl;
		m_graph.add_vertex(i,m_lambda2*(m_nodes[i].first-m_end_coord).norm());
		if(is_start(m_nodes[i])){
			//			std::cout<< "start_coord" << m_current_coord << std::endl;
			//			std::cout<<"start: "<< m_nodes[i].first << ", " << m_nodes[i].second <<std::endl;
			if(m_start_index!=-1){
				std::cout << "2 start nodes, fail" << std::endl;
				//return;
				//exit(1);
			}
			m_start_index=i;
		}
		if(is_goal(m_nodes[i])){
			//			std::cout<<"end: "<< m_nodes[i].first <<std::endl;
			if(m_end_index!=-1){
				std::cout << "2 end nodes, fail" << std::endl;
				//return;
				//exit(1);
			}
			m_end_index=i;
		}
	}
	if(m_start_index==-1){
		std::cout << "0 start node, fail" << std::endl;
		return;
	}
	if(m_end_index==-1){
		std::cout << "0 end node, fail" << std::endl;
		std::cout<< "goal :" << m_end_coord << std::endl;
		return;
	}
	for(int i=0;i<l;++i){
		for(int j=i+1;j<l;++j){
			if(neighboor(m_nodes[i],m_nodes[j])){
				//					std::cout<< "neighboor:" << i << "," << j <<std::endl;
				//					std::cout<< "cost:" << cost(i,j) <<std::endl;
				m_graph.add_edge(i,j,m_cost[j]);
				m_graph.add_edge(j,i,m_cost[i]);
			}
		}
	}
}


void MSP3D::Gfull(){
	m_graph.clear();
	m_nodes.clear();
	m_cost.clear();
	m_start_index=-1;
	m_end_index=-1;
	m_current_coord=m_start_coord;
	//Calcul of Gfull
	for(octomap::OcTree::leaf_iterator it = m_tree.begin_leafs(),	end=m_tree.end_leafs(); it!= end; ++it){
		if(it->getOccupancy()<1-m_epsilon){
			m_nodes.push_back(std::pair<octomap::point3d,double>(it.getCoordinate(),it.getSize()));
			m_cost.push_back(cost_func(it->getOccupancy(),1)*pow(it.getSize()/m_tree.getNodeSize(m_max_tree_depth),3));
		}
	}
	int l=m_nodes.size();

	std::cout<< "number of nodes: " << l << std::endl;
	for(int i=0;i<l;++i){
		// !!!!!!!!!!!!!  if not in path?
		//		std::cout<< "node " << i << ":" << m_nodes[i].first <<std::endl;
		m_graph.add_vertex(i,m_lambda2*(m_nodes[i].first-m_end_coord).norm());
		if(is_start(m_nodes[i])){
			std::cout<<"start: "<< m_nodes[i].first <<std::endl;
			if(m_start_index!=-1){
				std::cout << "2 start nodes, fail" << std::endl;
				//exit(1);
			}
			m_start_index=i;
		}
		if(is_goal(m_nodes[i])){
			std::cout<<"end: "<< m_nodes[i].first <<std::endl;
			if(m_end_index!=-1){
				std::cout << "2 end nodes, fail" << std::endl;
				//exit(1);
			}
			m_end_index=i;
		}
	}
	if(m_start_index==-1){
		std::cout << "0 start node, fail" << std::endl;
	}
	if(m_end_index==-1){
		std::cout << "0 end node, fail" << std::endl;
	}
	for(int i=0;i<l;++i){
		double progress=i*1.0/l;
		int barWidth = 70;

		std::cout << "[";
		int pos = barWidth * progress;
		for (int j = 0; j < barWidth; ++j) {
			if (j < pos) std::cout << "=";
			else if (j == pos) std::cout << ">";
			else std::cout << " ";
		}
		std::cout << "] " << int(progress * 100.0) << " %\r";
		std::cout.flush();
		for(int j=i+1;j<l;++j){
			if(neighboor(m_nodes[i],m_nodes[j])){
				//					std::cout<< "neighboor:" << i << "," << j <<std::endl;
				//					std::cout<< "cost:" << cost(i,j) <<std::endl;
				m_graph.add_edge(i,j,m_cost[j]);
				m_graph.add_edge(j,i,m_cost[i]);
			}
		}
	}
	std::cout << std::endl;
}

octomap::OcTreeNode* MSP3D::findNode(octomap::point3d pt){
	for(octomap::OcTree::tree_iterator it = m_tree.begin_tree(),	end=m_tree.end_tree(); it!= end; ++it)
	{
		if(it.getCoordinate()==pt){
			return &(*it);
		}
	}
	return NULL;
}

void MSP3D::copyNode(octomap::OcTreeNode* n,octomap::OcTreeNode* nc){
	nc->setLogOdds(n->getLogOdds());
	if(n->hasChildren()){
		for(int i=0;i<8;++i){
			if(n->childExists(i)){
				if(!(nc->childExists(i))){
					nc->createChild(i);
				}
				copyNode(n->getChild(i),nc->getChild(i));
			}
		}
	}
}

bool MSP3D::is_start(std::pair<octomap::point3d,double> &node){
	return is_in(m_current_coord,node);
}

bool MSP3D::is_goal(std::pair<octomap::point3d,double> &node){
	return is_in(m_end_coord,node);
}

bool MSP3D::is_in(octomap::point3d pt,std::pair<octomap::point3d,double> node){
	double l=0.5*node.second;
	if(fabs(pt.x()-node.first.x())<=l+m_eps && fabs(pt.y()-node.first.y())<=l+m_eps && fabs(pt.z()-node.first.z())<=l+m_eps){
		return true;
	}
	return false;
}

double MSP3D::cost(int i, int j){
	double F=findNode(m_nodes[j].first)->getOccupancy();
	//	std::cout<< "F: " << F << std::endl;
	if (F <= 1-m_epsilon/pow(m_nodes[j].second/m_tree.getResolution(),3)){
		//		std::cout << "normal cost: " << (m_lambda1*F+m_lambda2)*(1-pow(8,m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth))))/(1-8) << std::endl;
		//		std::cout << "node size: " << m_nodes[j].second << std::endl;
		//		std::cout << "tree resolution: " <<  m_tree.getResolution()*pow(2,16-m_max_tree_depth) << std::endl;
		//return (m_lambda1*F+m_lambda2)*(1-pow(8,m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth))))/(1-8);
		//return (m_lambda1*F+m_lambda2)*pow(8,m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth))-1);
		//return (m_lambda1*F+m_lambda2)*8*pow(m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth))-1,3);
		//return (m_lambda1*F+m_lambda2)*pow(m_nodes[j].second/(m_tree.getResolution()*pow(2,16-m_max_tree_depth)),3);
		return (m_lambda1*F+m_lambda2)*pow(m_nodes[j].second/m_tree.getNodeSize(m_max_tree_depth),3);
	}else{
		//		std::cout << "obstacle cost: " << m_M << std::endl;
		return m_M;
	}
}

/*double MSP3D::low_cost(octomap::point3d pt){
	return cost_func(findNode(pt)->getOccupancy());
	//std::cout << pt << F << std::endl;

}*/

double MSP3D::cost_func(double F,double size){
	if (F <= 1-m_epsilon/pow(size/m_tree.getResolution(),3)){
		return m_lambda1*F+m_lambda2;
	}else{
		return m_M;
	}
}

bool MSP3D::neighboor(std::pair<octomap::point3d,double> &na,std::pair<octomap::point3d,double> &nb){
	double l=0.5*(na.second+nb.second);
	int c=0;
	if(fabs(na.first.x()-nb.first.x())<=(l+m_eps) && fabs(na.first.y()-nb.first.y())<=(l+m_eps) && fabs(na.first.z()-nb.first.z())<=(l+m_eps)){
		if(fabs(na.first.x()-nb.first.x())<(l-m_eps)){
			++c;
		}
		if(fabs(na.first.y()-nb.first.y())<(l-m_eps)){
			++c;
		}
		if(fabs(na.first.z()-nb.first.z())<(l-m_eps)){
			++c;
		}
		if(c==2){
			return true;
		}
	}
	return false;
}

}
