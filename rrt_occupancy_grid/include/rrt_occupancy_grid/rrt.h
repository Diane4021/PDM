#ifndef rrt_h
#define rrt_h

#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"

using namespace std;
namespace rrt {
	class RRT{

        public:

            RRT();
            RRT(double input_PosX, double input_PosY);

            struct rrtNode{
                int nodeID;
                double posX;
                double posY;
                int parentID;
                vector<int> children;
            };

            vector<rrtNode> getTree();
            void setTree(vector<rrtNode> input_rrtTree);
            int getTreeSize();

            void addNewNode(rrtNode node);
            // rrtNode removeNode(int nodeID);
            rrtNode getNode(int nodeID);

            double getPosX(int nodeID);
            double getPosY(int nodeID);
            // void setPosX(int nodeID, double input_PosX);
            // void setPosY(int nodeID, double input_PosY);

            rrtNode getParent(int nodeID);
          //  void setParentID(int nodeID, int parentID);
            // void addChildID(int nodeID, int childID);
            // vector<int> getChildren(int nodeID);
            // int getChildrenSize(int nodeID);
            int getNearestNodeID(double X, double Y);
            vector<int> getRootToEndPath(int endNodeID);
            
        private:
            vector<rrtNode> rrtTree;
            
	};
};
double getEuclideanDistance(double sourceX, double sourceY, double destinationX, double destinationY);
//gridValue function prototype
int gridValue(nav_msgs::OccupancyGrid &,rrt::RRT::rrtNode);

//ObstacleFree function prototype
bool ObstacleFree(rrt::RRT::rrtNode ,rrt::RRT::rrtNode , nav_msgs::OccupancyGrid);
//check if the tempnode is inside the map
bool checkIfInsideBoundary(rrt::RRT::rrtNode tempNode);
//generate a temp node
void generateTempPoint(rrt::RRT::rrtNode &tempNode);

bool addNewPointtoRRT(rrt::RRT &myRRT, rrt::RRT::rrtNode &tempNode, float rrtStepSize, nav_msgs::OccupancyGrid mapsub);//tempnode changed to be the x_new

bool checkNodetoGoal(int X, int Y, rrt::RRT::rrtNode &tempNode);

//bool ObstacleFreepoint(rrt::RRT::rrtNode);
double DistanceBtwPairs(std::pair<double, double> coord1, std::pair<double, double> coord2);
#endif
