#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>


// global variables
nav_msgs::OccupancyGrid mapData;
visualization_msgs::Marker points;
std::string execute_cmd;


std::vector<geometry_msgs::PoseStamped> robots_pose;

//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData=*msg;
}


 
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)
{ 

geometry_msgs::Point p;  
p.x=msg->point.x;
p.y=msg->point.y;
p.z=msg->point.z;

points.points.push_back(p);

}



void dfsCallBack(const std_msgs::StringConstPtr& msg){
   execute_cmd = (*msg).data.c_str();
}

/*
void poseCallBack(const geometry_msgs::PoseStamped pose){
  //tf::StampedTransform transform;

  if (robots_pose.size()==0) {

    for (int i=0; i<3; i++) {
        robots_pose.push_back(pose);
    }
  }
  std::string global_frame = "/map_merged/map";
  if (pose.header.frame_id == "/robot_1") {
    try{
      std::string base = "/robot_1/base_link";
      listener.transformPoint(global_frame, pose, robots_pose[0]);
    } catch (tf::TransformException &ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      //continue;
    }
  } else if (pose.header.frame_id == "/robot_2") {
    try{
      listener.transformPoint(global_frame, pose, robots_pose[1]);
    } catch (tf::TransformException &ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      //continue;
    }
  } else if (pose.header.frame_id == "/robot_3") {
    try{
      listener.transformPoint(global_frame, pose, robots_pose[2]);
    } catch (tf::TransformException &ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      //continue;
    }
  }
} */


int main(int argc, char **argv)
{

  ros::init(argc, argv, "global_dfs_frontier_detector");
  ros::NodeHandle nh;


  //debug enabled
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  // fetching all parameters
  std::string map_topic,base_frame_topic, robot1_topic, robot2_topic, robot3_topic, detect_topic;
  
  std::string ns;
  ns=ros::this_node::getName();
  //ROS_INFO("%s",ns.c_str());

  tf::TransformListener listener;

  ros::param::param<std::string>(ns+"/map_topic", map_topic, "/map_merged/map");
  ros::param::param<std::string>(ns+"/cmd_topic", detect_topic, "/detect_cmd");

//---------------------------------------------------------------
  ros::Subscriber sub= nh.subscribe(map_topic, 1 ,mapCallBack);

  ros::Subscriber cmd_sub= nh.subscribe(detect_topic, 1, dfsCallBack);
  ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>(ns+"_shapes", 10);

  ros::Rate rate(100);
 
 
// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
while (mapData.header.seq<1 or mapData.data.size()<1)  {  ros::spinOnce();  ros::Duration(0.1).sleep();}

std::string global_frame = mapData.header.frame_id;
for (int i=0; i<3; i++) {
  listener.waitForTransform(global_frame, "robot_" + i, ros::Time(0), ros::Duration(10.0));
}

//visualizations  points and lines..
points.header.frame_id=mapData.header.frame_id;
points.header.stamp=ros::Time(0);
	
points.ns=line.ns = "dfsmarkers";
points.id = 0;


points.type = points.POINTS;

//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
points.action =points.ADD;
points.pose.orientation.w =1.0;
points.scale.x=0.3; 
points.scale.y=0.3; 

points.color.r = 255.0/255.0;
points.color.g = 0.0/255.0;
points.color.b = 0.0/255.0;
points.color.a=1.0;
points.lifetime = ros::Duration();

geometry_msgs::Point p;  


std::vector<float> frontiers;

// Main loop
while (ros::ok()){


  if (!execute_cmd.empty()) {
    nav_msgs::OccupancyGrid data = mapData;
    if (data.data.size()>0) {
        float Xstartx = data.info.origin.position.x;
        float Xstarty = data.info.origin.position.y;
        float resolution = data.info.resolution;
        unsigned int width = data.info.width;
        unsigned int height = data.info.height;


        for (int i=0; i<3; i++) {
            std::string base_frame = "/robot_" + i + "/base_link";
            tf::Transform transform;
            geometry_msgs::PointStamped robot1_pose = listener.lookupTransform(global_frame, base_frame, ros::Time(0), transform);
            unsigned int indx= (unsigned int)(floor((transform.getOrigin().getY()-Xstarty)/resolution)*width+ floor((transform.getOrigin().getX()-Xstartx)/resolution));

            std::vector<bool> visited(width*height, false);
            std::queue<unsigned int> queue;





        }


    }

  }



/*
// ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
char   checking=ObstacleFree(x_nearest,x_new,mapData);

    if (checking==-1){
            //geometry_msgs::PointStamped exploration_point;

            // the first point of a frontier
            p.x=x_new[0];
            p.y=x_new[1];
            p.z=0.0;

            ROS_DEBUG("before(%f, %f)", p.x, p.y);
            bool existFrontier = getCompleteFrontier(p, exploration_goal, mapData);
            if (!existFrontier) {
                exploration_goal.point.x = p.x;
                exploration_goal.point.y = p.y;

            }

            exploration_goal.header.stamp=ros::Time(0);
            exploration_goal.header.frame_id=mapData.header.frame_id;
            //exploration_goal.point.x=x_new[0];
            //exploration_goal.point.y=x_new[1];
            exploration_goal.point.z=0.0;

            //p.x = exploration_goal.point.x;
            //p.y = exploration_goal.point.y;
            points.points.push_back(p);

            ROS_DEBUG("after(%f, %f)", p.x, p.y);

            pub.publish(points) ;
            targetspub.publish(exploration_goal);
            points.points.clear();

          }
	  	
	  
	  else if (checking==1){
	 	V.push_back(x_new);
	 	
	 	p.x=x_new[0]; 
		p.y=x_new[1]; 
		p.z=0.0;
	 	line.points.push_back(p);
	 	p.x=x_nearest[0]; 
		p.y=x_nearest[1]; 
		p.z=0.0;
	 	line.points.push_back(p);

	        }



pub.publish(line);  

   

    ros::spinOnce();
    rate.sleep();
    }
    */
    return 0;
}
