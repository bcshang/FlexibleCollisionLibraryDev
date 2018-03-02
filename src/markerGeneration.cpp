#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>


#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>

#define jointDiameter .2

/**
 * Returns the distance between two points
 * @param  point1 first point
 * @param  point2 second point
 * @return        distance between points
 */
double calcDistance(sejong::Vect3 point1, sejong::Vect3 point2) {
  double distance1 = pow((point1[0] - point2[0]), 2);
  double distance2 = pow((point1[1] - point2[1]), 2);
  double distance3 = pow((point1[2] - point2[2]), 2);

  return sqrt(distance1 + distance2 + distance3);
}

/**
 * Calculate the midpoint between two points
 * @param  point1 first point
 * @param  point2 second point
 * @return        midpoint of the two points
 */
sejong::Vect3 calcMidpoint(sejong::Vect3 point1, sejong::Vect3 point2) {
  return sejong::Vect3((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2, (point1[2] + point2[2]) / 2);
}



/**
 * Creates a ROS visualization marker
 * @param x coordinate of marker
 * @param y coordinate of marker
 * @param z coordinate of marker
 */
visualization_msgs::Marker createMarker(double x_pos, double y_pos, double z_pos) {
  visualization_msgs::Marker marker;
  static int markernum = 0;

  int shape = visualization_msgs::Marker::CUBE;


  marker.header.frame_id = "/val_robot/pelvis";
  marker.header.stamp = ros::Time::now();

  marker.ns = "basic_shapes";
  marker.id = markernum;
  markernum++;

  // Set the marker shape
  marker.type = shape;

  // Set the marker action
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker
  marker.pose.position.x = x_pos;
  marker.pose.position.y = y_pos;
  marker.pose.position.z = z_pos;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();


  return marker;
}



/**
 * Convenience wrapper for markers
 * @param  pos position of the marker
 * @return     ROS Marker message
 */
visualization_msgs::Marker createMarker(sejong::Vect3 pos) {
  return createMarker(pos[0], pos[1], pos[2]);
}

/**
 * Create a marker cylinder for visualization purposes
 * @param  pos1 bottom point
 * @param  pos2 top point
 * @return      ROS Marker message
 */
visualization_msgs::Marker createCylinder(sejong::Vect3 pos1, sejong::Vect3 pos2) {
  visualization_msgs::Marker marker;
  double dist = calcDistance(pos1, pos2);
  sejong::Vect3 midpoint = calcMidpoint(pos1, pos2);

  static int markernum = 0;

  int shape = visualization_msgs::Marker::CYLINDER;


  marker.header.frame_id = "/val_robot/pelvis";
  marker.header.stamp = ros::Time::now();

  marker.ns = "basic_shapes";
  marker.id = markernum;
  markernum++;

  // Set the marker shape
  marker.type = shape;

  // Set the marker action
  marker.action = visualization_msgs::Marker::ADD;

  // Set the position of the marker
  marker.pose.position.x = midpoint[0];
  marker.pose.position.y = midpoint[1];
  marker.pose.position.z = midpoint[2];

  // set up a quaternion if we want to
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = jointDiameter;
  marker.scale.y = jointDiameter;
  marker.scale.z = dist;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  return marker;
}

