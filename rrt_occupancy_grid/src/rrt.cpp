#include <rrt_occupancy_grid/rrt.h>
#include <math.h>
#include <cstddef>
#include <iostream>

using namespace rrt;

/**
* default constructor for RRT class
* initializes source to 0,0
* adds sorce to rrtTree
*/
RRT::RRT()
{
    RRT::rrtNode newNode;
    newNode.posX = 0;
    newNode.posY = 0;
    newNode.parentID = 0;
    newNode.nodeID = 0;
    rrtTree.push_back(newNode);
}

/**
* default constructor for RRT class
* initializes source to input X,Y
* adds sorce to rrtTree
*/
RRT::RRT(double input_PosX, double input_PosY)
{
    RRT::rrtNode newNode;
    newNode.posX = input_PosX;
    newNode.posY = input_PosY;
    newNode.parentID = 0;
    newNode.nodeID = 0;
    rrtTree.push_back(newNode);
}

/**
* Returns the current RRT tree
*/
vector<RRT::rrtNode> RRT::getTree()
{
    return rrtTree;
}
// For setting the rrtTree to the inputTree

void RRT::setTree(vector<RRT::rrtNode> input_rrtTree)
{
    rrtTree = input_rrtTree;
}
//use property of vector  to get the number of nodes in the rrt Tree 
//.size()
int RRT::getTreeSize()
{
    return rrtTree.size();
}
//adding a new node to the rrt Tree
void RRT::addNewNode(RRT::rrtNode node)
{
    rrtTree.push_back(node);
}
// //removing a node from tree
// RRT::rrtNode RRT::removeNode(int id)
// {
//     RRT::rrtNode tempNode = rrtTree[id];
//     rrtTree.erase(rrtTree.begin()+id);
//     return tempNode;
// }
//use node id to get a specific node
RRT::rrtNode RRT::getNode(int id)
{
    return rrtTree[id];
}
//return a existing node from the rrt tree nearest to the given point
int RRT::getNearestNodeID(double X, double Y)
{
    int i, returnID;
    double distance = 9999, tempDistance;
    for(i=0; i<this->getTreeSize(); i++)
    {
        tempDistance = getEuclideanDistance(X,Y, getPosX(i),getPosY(i));
        if (tempDistance < distance)
        {
            distance = tempDistance;
            returnID = i;
        }
    }
    return returnID;
}
 //returns X coordinate of the given node
double RRT::getPosX(int nodeID)
{
    return rrtTree[nodeID].posX;
}
 //returns Y coordinate of the given node
double RRT::getPosY(int nodeID)
{
    return rrtTree[nodeID].posY;
}
 //returns parentID of the given node
RRT::rrtNode RRT::getParent(int id)
{
    return rrtTree[rrtTree[id].parentID];
}

 //set parentID of the given node

// void RRT::setParentID(int nodeID, int parentID)
// {
//     rrtTree[nodeID].parentID = parentID;
// }

/**
// * add a new childID to the children list of the given node
// */
// void RRT::addChildID(int nodeID, int childID)
// {
//     rrtTree[nodeID].children.push_back(childID);
// }

// /**
// * returns the children list of the given node
// */
// vector<int> RRT::getChildren(int id)
// {
//     return rrtTree[id].children;
// }

// /**
// * returns number of children of a given node
// */
// int RRT::getChildrenSize(int nodeID)
// {
//     return rrtTree[nodeID].children.size();
// }
/**
* returns euclidean distance between two set of X,Y coordinates
*/
double getEuclideanDistance(double sourceX, double sourceY, double destinationX, double destinationY)
{
    return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2));
}
/**
* returns path from root to end node
* @param endNodeID of the end node
* @return path containing ID of member nodes in the vector form
*/
vector<int> RRT::getRootToEndPath(int endNodeID)
{
    vector<int> path;
    path.push_back(endNodeID);
    while(rrtTree[path.front()].nodeID != 0)
    {
        //std::cout<<rrtTree[path.front()].nodeID<<endl;
        path.insert(path.begin(),rrtTree[path.front()].parentID);
    }
    return path;
}

//gridValue function known the point position Xp, want to know the grid value of the position
int gridValue(nav_msgs::OccupancyGrid &mapData, RRT::rrtNode tempnode){
/*
MapMetaData
time map_load_time
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin*/
float resolution=mapData.info.resolution;
float Xstartx=mapData.info.origin.position.x;
float Xstarty=mapData.info.origin.position.y;//Xstaty is the origin of the map, not the begining point we chose

float width=mapData.info.width;
std::vector<signed char> Data=mapData.data;//the probabilities of each cell
double posX,posY;
posX=tempnode.posX;
posY=tempnode.posY;
//returns grid value at "Xp" location
//map data:  100 occupied      -1 unknown       0 free 
float indx=(  floor((posY-Xstarty)/resolution)*width)+( floor((posX-Xstartx)/resolution) );//floor:Rounds x downward, returning the largest integral value that is not greater than x.

int out;
//std::cout<<"indx"<<indx;
out=Data[int(indx)];
return out;
}


//ObstacleFree function
bool ObstacleFree(RRT::rrtNode xnear, RRT::rrtNode xnew, nav_msgs::OccupancyGrid mapsub){
  //if xnear=xnearest, to check whether there are obstacles on the trajectory
float rez=float(mapsub.info.resolution)*0.5;//choosing a steplength 
float distance;
distance=getEuclideanDistance(xnear.posX,xnear.posY ,xnew.posX,xnew.posY);
float stepz=int(ceil(distance/rez)); //number of steps to check, function ceiling
RRT::rrtNode xi=xnear;//at start,xi=xnearst
int  obs=0; int unk=0;//to check if the node is ina unkown or obstacle filed

double theta = atan2(xnew.posY - xnear.posY,xnew.posX - xnear.posX);
//geometry_msgs::Point p;
for (int c=0;c<stepz+1;c++){//std::vector<float> Steer(  std::vector<float> x_nearest , std::vector<float> x_rand, float eta)
 
  xi.posX = xi.posX + (rez * cos(theta));
  xi.posY = xi.posY + (rez * sin(theta));
 
   if (gridValue(mapsub,xi) ==100){     obs=1; break;}//int gridValue(nav_msgs::OccupancyGrid &mapData,std::vector<float> Xp)
   
   if (gridValue(mapsub,xi) ==-1){      unk=1;	break;}
  }
bool out=true;
 if (unk==1||obs==1)
 {
     out=false;
 }
 return out;

 }

 bool checkIfInsideBoundary(RRT::rrtNode tempNode)
{
    // if(tempNode.posX < 0 || tempNode.posY < -7  || tempNode.posX > 6 || tempNode.posY > 0 ) return false;
    return true;
}

void generateTempPoint(RRT::rrtNode &tempNode)
{
    double x = ((rand()%101)/101.1)*7-0.5; //in range 0:6
    double y = -((rand()%101)/101.1)*7+0.5; //in range -7:0
    // std::cout<<"Random X: "<<x <<endl<<"Random Y: "<<y<<endl;
    tempNode.posX = x;
    tempNode.posY = y;
}

bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, float rrtStepSize, nav_msgs::OccupancyGrid mapsub)//tempnode changed to be the x_new
{
    int nearestNodeID = myRRT.getNearestNodeID(tempNode.posX,tempNode.posY);

    RRT::rrtNode nearestNode = myRRT.getNode(nearestNodeID);

    // double theta = atan2(tempNode.posY - nearestNode.posY,tempNode.posX - nearestNode.posX);


    // tempNode.posX = nearestNode.posX + (rrtStepSize * cos(theta));
    // tempNode.posY = nearestNode.posY + (rrtStepSize * sin(theta));

    if(checkIfInsideBoundary(tempNode) &&ObstacleFree(nearestNode, tempNode, mapsub))
    {
        tempNode.parentID = nearestNodeID;
        tempNode.nodeID = myRRT.getTreeSize();
        nearestNode.children.push_back(tempNode.nodeID);
        myRRT.addNewNode(tempNode);
        // std::cout<<"tempNode.nodeID: "<<tempNode.nodeID <<endl<<"parentID "<<tempNode.parentID <<endl;
        //std::cout<<"new x"<<tempNode.posX <<endl<<"parentID "<<tempNode.posY<<endl;//XY未更新
        return true;
    }
    else
        return false;//if the node in the boudnary of map and the trajectory not collide with obstacles
}

bool checkNodetoGoal(int X, int Y, RRT::rrtNode &tempNode)
{
    double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2));
    if(distance < 0.5)//setting a threshold
    {
        return true;
    }
    return false;
}

double DistanceBtwPairs(std::pair<double, double> coord1, std::pair<double, double> coord2)
{
    double distance = sqrt(pow((coord1.first - coord2.first),2) + pow((coord1.second - coord2.second),2));
    return distance;
}