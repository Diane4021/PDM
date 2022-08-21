#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <rrt_occupancy_grid/rrt.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include<cstdlib>
#include <utility>
#include <chrono>



#define success false
#define running true

using namespace std::chrono;

using namespace rrt;

bool status = running;


nav_msgs::OccupancyGrid my_map;//a global variable to store the map information
// a call back function
void get_metadata(const nav_msgs::OccupancyGrid map)
{
    my_map=map;//store the map information
    ROS_INFO_STREAM("topic data found:"<<map.info.resolution);
    ROS_INFO_STREAM("received:"<<my_map.info.resolution);
}

void initializeMarkers(visualization_msgs::Marker &sourcePoint,
    visualization_msgs::Marker &goalPoint,
    visualization_msgs::Marker &randomPoint,
    visualization_msgs::Marker &rrtTreeMarker,
    visualization_msgs::Marker &bestPath)
{
    //init headers
	sourcePoint.header.frame_id    = goalPoint.header.frame_id    = randomPoint.header.frame_id    = rrtTreeMarker.header.frame_id    = bestPath.header.frame_id    = "/map";
	sourcePoint.header.stamp       = goalPoint.header.stamp       = randomPoint.header.stamp       = rrtTreeMarker.header.stamp       = bestPath.header.stamp       = ros::Time::now();
	sourcePoint.ns                 = goalPoint.ns                 = randomPoint.ns                 = rrtTreeMarker.ns                 = bestPath.ns                 = "path_planner";
	sourcePoint.action             = goalPoint.action             = randomPoint.action             = rrtTreeMarker.action             = bestPath.action             = visualization_msgs::Marker::ADD;
	sourcePoint.pose.orientation.w = goalPoint.pose.orientation.w = randomPoint.pose.orientation.w = rrtTreeMarker.pose.orientation.w = bestPath.pose.orientation.w = 1.0;

    //setting id for each marker
    sourcePoint.id    = 0;
	goalPoint.id      = 1;
	randomPoint.id    = 2;
	rrtTreeMarker.id  = 3;
    bestPath.id      = 4;

	//defining types
	rrtTreeMarker.type                                    = visualization_msgs::Marker::LINE_LIST;
	bestPath.type                                        = visualization_msgs::Marker::LINE_STRIP;
	sourcePoint.type  = goalPoint.type = randomPoint.type = visualization_msgs::Marker::SPHERE;

	//setting scale
	rrtTreeMarker.scale.x = 0.02;
	bestPath.scale.x     = 0.03;
	sourcePoint.scale.x   = goalPoint.scale.x = randomPoint.scale.x = 0.2;
    sourcePoint.scale.y   = goalPoint.scale.y = randomPoint.scale.y = 0.2;
    sourcePoint.scale.z   = goalPoint.scale.z = randomPoint.scale.z = 0.1;

    //assigning colors
	sourcePoint.color.r   = 1.0f;
	goalPoint.color.g     = 1.0f;
    randomPoint.color.b   = 1.0f;

	rrtTreeMarker.color.r = 0.8f;
	rrtTreeMarker.color.g = 0.4f;

	bestPath.color.r = 0.2f;
	bestPath.color.g = 0.2f;
	bestPath.color.b = 1.0f;

	sourcePoint.color.a = goalPoint.color.a = randomPoint.color.a = rrtTreeMarker.color.a = bestPath.color.a = 1.0f;
}

void addBranchtoRRTTree(visualization_msgs::Marker &rrtTreeMarker, RRT::rrtNode &tempNode, RRT &myRRT)
{

geometry_msgs::Point point;

point.x = tempNode.posX;
point.y = tempNode.posY;
point.z = 0;
rrtTreeMarker.points.push_back(point);

RRT::rrtNode parentNode = myRRT.getParent(tempNode.nodeID);

point.x = parentNode.posX;
point.y = parentNode.posY;
point.z = 0;

rrtTreeMarker.points.push_back(point);
}


void setbestPathData(vector< vector<int> > &rrtPaths, RRT &myRRT, int i, visualization_msgs::Marker &bestpath, int goalX, int goalY)
{
    bestpath.points.clear();
    RRT::rrtNode tempNode;
    geometry_msgs::Point point;
    for(int j=0; j<rrtPaths[i].size();j++)
    {
        tempNode = myRRT.getNode(rrtPaths[i][j]);

        point.x = tempNode.posX;
        point.y = tempNode.posY;
        point.z = 0;

        bestpath.points.push_back(point);
    }

    point.x = goalX;
    point.y = goalY;
    bestpath.points.push_back(point);
}//printing the final path


double ComputePathLength(vector<int> path, RRT &myRRT, int goalX, int goalY)
{
    //first convert indices to xyz coords
    vector<std::pair<double, double>> coord_path;
    std::pair<double, double> coord;
    RRT::rrtNode node;
    for (int i = 0; i < path.size(); i++)
    {
        node = myRRT.getNode(path[i]);
        coord.first = node.posX;
        coord.second = node.posY;
        coord_path.push_back(coord);
    }
    coord.first = goalX;
    coord.second = goalY;
    coord_path.push_back(coord);

    //calculate length
    double length = 0.;
    for (int i = 0; i < coord_path.size()-1; i++)
    {
        length+=DistanceBtwPairs(coord_path[i], coord_path[i+1]);
    }
    return length;
}


int main(int argc,char** argv)
{
    //initializing ROS
    ros::init(argc,argv,"rrt_occupancy_grid_node");
	ros::NodeHandle n;
    
    //Let ROS takeover
   
	//defining Publisher
	ros::Publisher rrt_publisher = n.advertise<visualization_msgs::Marker>("path_planner_rrt",1);
    
	//defining markers
    visualization_msgs::Marker sourcePoint;
    visualization_msgs::Marker goalPoint;
    visualization_msgs::Marker randomPoint;
    visualization_msgs::Marker rrtTreeMarker;
    visualization_msgs::Marker bestPath;

    initializeMarkers(sourcePoint, goalPoint, randomPoint, rrtTreeMarker, bestPath);//at top

    //setting source and goal
    sourcePoint.pose.position.x = 0;
    sourcePoint.pose.position.y = 0;

    goalPoint.pose.position.x = 0;
    goalPoint.pose.position.y = -6;

    rrt_publisher.publish(sourcePoint);//name of node
    rrt_publisher.publish(goalPoint);
    ros::spinOnce();
    ros::Duration(0.01).sleep();

    //srand (time(NULL));
    //initialize rrt specific variables

    //initializing rrtTree
    RRT myRRT(0,0); ///rrt.cpp all the methodes of RRT start point
    double goalX, goalY;
    goalX= goalPoint.pose.position.x;
    goalY = goalPoint.pose.position.y;

    float rrtStepSize = 0.3;

    vector< vector<int> > rrtPaths;//all paths
    vector<int> path;//one path
    int rrtPathLimit = 30;

    // int shortestPathLength = 9999;
    int shortestPath = 0;
  
   // vector< vector<geometry_msgs::Point> >  obstacleList = getObstacles();//a matrix //line68
    ros::Subscriber sub_map=n.subscribe("map",3,&get_metadata);
    ros::Duration(3).sleep();//sleep for 3 secs waiting for message

    ros::spinOnce();

    bool addNodeResult = false, nodeToGoal = false;
    ROS_INFO_STREAM("data received:"<<my_map.info.width);

    RRT::rrtNode tempNode;//a structure
    auto start = high_resolution_clock::now();
    while(ros::ok() && status)//status=running line19
    {
        if(rrtPaths.size() < rrtPathLimit)// this finds all paths up to limit
        {
            
            generateTempPoint(tempNode);//create a temp node
            //std::cout<<"tempnode generated"<<tempNode.posX<<endl;
            addNodeResult = addNewPointtoRRT(myRRT,tempNode,rrtStepSize,my_map); //true if no obstacles
            //bool addNewPointtoRRT(RRT &myRRT, RRT::rrtNode &tempNode, float rrtStepSize, nav_msgs::OccupancyGrid mapsub)
            //tempnode changed to be the x_new
            if(addNodeResult)//if the x_new can be added
            {
                addBranchtoRRTTree(rrtTreeMarker,tempNode,myRRT);//printing in rviz
                nodeToGoal = checkNodetoGoal(goalX, goalY,tempNode);
                if(nodeToGoal)
                {
                    path = myRRT.getRootToEndPath(tempNode.nodeID);//get the path that we just found
                    rrtPaths.push_back(path);
                    std::cout<<"New path found with length: "<<ComputePathLength(path, myRRT, goalX, goalY)<<". Total paths: "<<rrtPaths.size()<<endl;
                    if (rrtPaths.size()==1||rrtPaths.size()==30)
                    {
                        auto stop = high_resolution_clock::now();
                        auto duration = duration_cast<microseconds>(stop - start);
                        cout << "Runtime [s]: "<< duration.count()*pow(10,-6) << endl;
                    }

                    for(int i=0; i<rrtPaths.size();i++)//for rviz
                    {
                        if(ComputePathLength(rrtPaths[i], myRRT, goalX, goalY) <= ComputePathLength(rrtPaths[shortestPath], myRRT, goalX, goalY))\
                        {
                            shortestPath = i;

                            std::cout<<"Path "<<i<<" is the shortest path with length: "<<ComputePathLength(rrtPaths[i], myRRT, goalX, goalY)<<endl;
                            std::cout<<"It spans ... nodes "<<myRRT.getTreeSize()<<endl;
                        }
                    }
                    setbestPathData(rrtPaths, myRRT, shortestPath, bestPath, goalX, goalY);//printing teh final path
                    rrt_publisher.publish(bestPath);//visualization_msgs::Marker bestPath;
                }
            }
        //    for(int i=0;i<path.size();i++)
        // {
        //     std::cout<<"path id"<<path[i]<<endl;//printing the path found
        // }  
        }
        else
        {
            setbestPathData(rrtPaths, myRRT, shortestPath, bestPath, goalX, goalY);
        }
       

        rrt_publisher.publish(sourcePoint);
        rrt_publisher.publish(goalPoint);
        rrt_publisher.publish(rrtTreeMarker);
        //rrt_publisher.publish(bestPath);
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

	return 1;
}
