#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// flags for robot pick up and drop off action
bool pick_up = false;
bool drop_off = false;

// pick up marker location
float marker_pick_x = 0.58;
float marker_pick_y = 3.33;
float marker_drop_x = -0.5;
float marker_drop_y = -0.5;

// define position threshold for checking distance between robot and marker
float dis_thresh = 0.5;

// define odom callback function.
// The function checks if robot is at pick up or drop off location
// Then, set pick_up and drop_off flag
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{


  // extract robot x and y position
  float robot_x = msg->pose.pose.position.x;
  float robot_y = msg->pose.pose.position.y; 
  
  // at startup, pick_up and drop_off are false.
  // When robot moves to pick_up location, set pick_up flag to true
  if(pick_up == false && drop_off == false)
  {
    // calculate distance between robot and pick_up location
    float dis_pick = sqrt((robot_x-marker_pick_x)*(robot_x-marker_pick_x) + (robot_y-marker_pick_y)*(robot_y-marker_pick_y));
    
      
    // if robot to pick_up location is smaller than threshold, set pick_up flag to true
    if(dis_pick<=dis_thresh)
    {
      ROS_INFO("pick up marker");
      pick_up = true;
    }
    
  }
  if(pick_up == true && drop_off == false)
  {
    // calculate distance between robot and drop_off location
    float dis_drop = sqrt((robot_x-marker_drop_x)*(robot_x-marker_drop_x) + (robot_y-marker_drop_y)*(robot_y-marker_drop_y));
      
    // if robot to drop_off location is smaller than threshold, set drop_off flag to true
    if(dis_drop<=dis_thresh)
    {
      ROS_INFO("drop off marker");
      drop_off = true;
    }
  }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odom_cb);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_pick_x;
    marker.pose.position.y = marker_pick_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    if(pick_up == true && drop_off == false)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      ros::Duration(2.0).sleep();
    }
    if(pick_up == true && drop_off == true)
    {
      marker.pose.position.x = marker_drop_x;
      marker.pose.position.y = marker_drop_y;
      marker.action = visualization_msgs::Marker::ADD;
//       marker_pub.publish(marker);
//       pick_up = true;
//       drop_off = false;
//       ros::Duration(5.0).sleep();
      ros::Duration(2.0).sleep();
    }
    marker_pub.publish(marker);
//     r.sleep();
    ros::spinOnce();
  }
  return 0;
}