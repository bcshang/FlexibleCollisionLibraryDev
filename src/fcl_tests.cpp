// Standard C things
#include <iostream>
#include <vector>
#include <math.h>
#include <cstring>

// ROS things
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include "geometry_msgs/Pose.h"
#include <interactive_markers/interactive_marker_server.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


// FCL things
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/narrowphase/collision_result.h"

#include "test_fcl_utility.h"

// Valkyrie things
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include "Valkyrie_Definition.h"
#include "RobotCollisionChecker.hpp"

// Helpful things
#include "markerGeneration.hpp"


#define NODE_RATE 15
#define sizeBox .25





fcl::CollisionObject<double> *colliderObstacle;

// TODO figure out how to make it so that all things are persistent without using "new"
// size is 5cm
fcl::CollisionObject<double>* generateObstacle(sejong::Vect3 location, double size) {
  // declare a box
  fcl::Box<double> *obstacleCube = new fcl::Box<double>(size, size, size);
  
  //fcl::BVHModel<fcl::OBBRSS<double>> *colliderModel = new fcl::BVHModel<fcl::OBBRSS<double>>();
  
  fcl::Transform3<double> obstacleTransform = fcl::Transform3<double>(fcl::Translation3<double>(location));
  
  // generateBVHModel(*colliderModel, obstacleCube, obstacleTransform);
  std::shared_ptr<fcl::CollisionGeometry<double>> colGeom(obstacleCube);
  std::string *str = new std::string("box");
  colGeom->setUserData(str);
  fcl::CollisionObject<double>* colliderObject = new fcl::CollisionObject<double>(colGeom, obstacleTransform);
  colliderObject->setUserData(str);
  return colliderObject;
}

void colliderCallback(const geometry_msgs::Pose& msg) {
  delete colliderObstacle;
  // need to fix for how fcl vs ros denotes markers
  colliderObstacle = generateObstacle(sejong::Vect3(msg.position.x, msg.position.y, msg.position.z-sizeBox/2), sizeBox);
}



int main(int argc, char** argv) {

  // Initialize a ROS node
  ros::init(argc, argv, "fcl_tests");
  ros::NodeHandle n;

  // temporary declaration
  colliderObstacle = generateObstacle(sejong::Vect3(100, 100, 100), .25);

  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  ros::Subscriber sub = n.subscribe("interactiveMarker", 10, colliderCallback);
  ros::Rate r(NODE_RATE);



  // Get Valkyrie information
  sejong::Vector m_q; m_q.resize(NUM_Q); m_q.setZero();
  sejong::Vector m_qdot; m_qdot.resize(NUM_QDOT); m_qdot.setZero();

  m_q[NUM_QDOT] = 1.0;

  RobotCollisionChecker<double> *valkyrie_collision_checker = new RobotCollisionChecker<double>(m_q, m_qdot);
  std::cout << "Robot collision Checker generated" << std::endl;
  valkyrie_collision_checker->generateRobotCollisionModel();
  std::cout << "Valkyrie FCL Model Constructed" << std::endl;

  // Variable size array
  std::vector<visualization_msgs::Marker> markerVector;

  // create markers based on the calculated link positions
  //markerVector.push_back(createMarker(left_foot_pos));
  //markerVector.push_back(createMarker(left_knee_pos));
  //markerVector.push_back(createCylinder(left_foot_pos, left_knee_pos));

  // Set up array markers message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.resize(markerVector.size());

  // add markers to message
  for(int i=0; i<markerVector.size(); i++) {
    markerArray.markers[i] = markerVector[i];
  }


  std::cout << "ROS Loop start" << std::endl;
  while(ros::ok()) {

    // Publish the marker array
    marker_pub.publish(markerArray); 
    
    ros::spinOnce();

    // std::cout << "Debug1" << std::endl;
    std::vector<fcl::Contact<double>> collisionContacts = valkyrie_collision_checker->collideWith(colliderObstacle);
    // std::cout << "Debug2" << std::endl;
    
    if(collisionContacts.size() > 0) {
      fcl::Contact<double> con = collisionContacts[0];
      std::cout << *((std::string*)con.o1->getUserData()) << std::endl;
      std::cout << ((CollisionLink<double>*)con.o2->getUserData())->link1 << std::endl;
      std::cout << "Blarg" << std::endl;
    }
    // std::cout << "Debug3" << std::endl;
    r.sleep();
  }

  return -1;
}



