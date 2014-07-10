
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <deque>
#include <map>
#include <vector>
#include <utility>
#include <string>
#include "kshortestpaths/Graph.h"
#include <ros/ros.h>

class Point3D_Less :
public binary_function<octomap::point3d, octomap::point3d, bool> {
	public :
	bool operator()(const octomap::point3d &lhs, const octomap::point3d& rhs) const{
    	if(lhs.x()<rhs.x())
			return 1;
		if(lhs.x()>rhs.x())
			return 0;
    	if(lhs.y()<rhs.y())
			return 1;
		if(lhs.y()>rhs.y())
			return 0;
    	if(lhs.z()<rhs.z())
			return 1;
    	else
    		return 0;
    }
};

namespace msp{
	class MSP3D{
		public:
			MSP3D(octomap::OcTree &tree,int max_depth);
			bool init(octomap::point3d start,octomap::point3d end);
			bool step();
			bool run();
			bool runAs();
			std::deque<octomap::point3d> getPath();
			double getPathCost();
			void setAlpha(double a){m_alpha=a;}
			void setSpeedUp(bool a){m_speed_up=a;}
			std::vector<octomap::point3d> m_child_dir;
			void setGiPublisher(ros::Publisher pub);
			void setPathPublisher(ros::Publisher pub){m_rviz_traj_pub=pub;}

		protected:
			void publishGiRviz();
			bool inPath(octomap::point3d pt,double size);
			octomap::OcTreeNode* findNode(octomap::point3d pt);
			bool findLRNode(octomap::point3d& pt,octomap::OcTreeKey& key, octomap::point3d& coord);
			//double low_cost(octomap::point3d pt);
			void reducedGraph();
			void Gfull();
			void iterativeReducedGraph(octomap::OcTree::NodeType *n);
			bool neighboor(std::pair<octomap::point3d,double> &na,std::pair<octomap::point3d,double> &nb);
			bool is_start(std::pair<octomap::point3d,double> &node);
			bool is_goal(std::pair<octomap::point3d,double> &node);
			bool is_in(octomap::point3d pt,std::pair<octomap::point3d,double> node);
			double cost(int i, int j);
			double cost_func(double F,double size);
			void copyNode(octomap::OcTreeNode* n,octomap::OcTreeNode* nc);
			void add_node_to_reduced_vertices(octomap::OcTreeNode* node,octomap::point3d coord, double size);
			octomap::OcTreeKey m_start;
			octomap::OcTreeKey m_end;
			octomap::point3d m_start_coord;
			octomap::point3d m_end_coord;
			//octomap::OcTreeKey m_current_point;
			octomap::point3d m_current_coord;
			octomap::OcTree m_tree;
			int m_start_index;
			int m_nb_backtrack;
			int m_end_index;
			bool m_speed_up;
			bool m_visu;
			bool m_path_found;
			std::deque<octomap::point3d> m_current_path;
			kshortestpaths::Graph m_graph;
			std::map<octomap::point3d,double,Point3D_Less> m_visited;
			std::map<octomap::point3d,std::set<octomap::point3d,Point3D_Less>,Point3D_Less> m_misleading;
			std::set<octomap::point3d,Point3D_Less> m_current_forbidden;
			double m_alpha;//used in reduced graph as parameter for decomposition
			std::vector<std::pair<octomap::point3d,double> > m_nodes; //coord,size
			std::vector<double> m_cost;
			std::vector<double> m_path_cost;
			std::vector<double> m_full_path_estimate_cost;
			std::vector<std::pair<octomap::point3d,double> > m_obstacles; //for debugging and display only
			double m_eps;//used as margin for comparison (small compared to tree resolution)
			double m_epsilon;
			double m_M;
			double m_lambda1;
			double m_lambda2;
			int m_max_tree_depth;
			int m_nb_step;
			ros::Publisher m_pub;
			ros::Publisher m_rviz_traj_pub;
	};
}
