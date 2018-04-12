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


#define NODE_RATE 10
#define sizeBox .25





fcl::CollisionObject<double> *colliderObstacle;
std::string *boxString;
// TODO figure out how to make it so that all things are persistent without using "new"
// size is 5cm
fcl::CollisionObject<double>* generateObstacle(sejong::Vect3 location, double size) {
  // declare a box
  fcl::Box<double> *obstacleCube = new fcl::Box<double>(size, size, size);
  
  //fcl::BVHModel<fcl::OBBRSS<double>> *colliderModel = new fcl::BVHModel<fcl::OBBRSS<double>>();
  
  fcl::Transform3<double> obstacleTransform = fcl::Transform3<double>(fcl::Translation3<double>(location));
  
  // generateBVHModel(*colliderModel, obstacleCube, obstacleTransform);
  std::shared_ptr<fcl::CollisionGeometry<double>> colGeom(obstacleCube);
  // std::string *str = new std::string("box");
  colGeom->setUserData(boxString);
  fcl::CollisionObject<double>* colliderObject = new fcl::CollisionObject<double>(colGeom, obstacleTransform);
  colliderObject->setUserData(boxString);
  return colliderObject;
}

void colliderCallback(const geometry_msgs::Pose& msg) {
  // delete colliderObstacle->getUserData();
  delete colliderObstacle;
  // need to fix for how fcl vs ros denotes markers
  colliderObstacle = generateObstacle(sejong::Vect3(msg.position.x, msg.position.y, msg.position.z-sizeBox/2), sizeBox);
}



void printCollisions(std::vector<fcl::Contact<double>> collisionContacts) {
 std::cout << "Number of collisions: " << collisionContacts.size() << std::endl;
 if(collisionContacts.size() > 0) {
    for(int i=0; i<collisionContacts.size(); i++) {
      fcl::Contact<double> con = collisionContacts[i];
      std::cout << "Collision Between: ";
      std::cout << *((std::string*)con.o1->getUserData());
      std::cout << " and joint ";
      std::cout << ((CollisionLink<double>*)con.o2->getUserData())->link1 << "-" << ((CollisionLink<double>*)con.o2->getUserData())->link2 << std::endl;
    }
  }
  else{
    std::cout << "No collisions" << std::endl;
  }
}



int realmain(int argc, char** argv) {

  // Initialize a ROS node
  ros::init(argc, argv, "fcl_tests");
  ros::NodeHandle n;

  // temporary declaration
  colliderObstacle = generateObstacle(sejong::Vect3(100, 100, 100), .25);
  boxString = new std::string("box");

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


  std::vector<visualization_msgs::Marker> markerVector = valkyrie_collision_checker->generateMarkers();

// Debugging things
  sejong::Vect3 vec;
  sejong::Vect3 vec1;
  
  valkyrie_collision_checker->robot_model->getPosition(m_q, SJLinkID::LK_rightShoulderPitchLink, vec);
  sejong::pretty_print(vec, std::cout, "R Shoulder pitch link");
  valkyrie_collision_checker->robot_model->getPosition(m_q, SJLinkID::LK_rightElbowPitchLink, vec1);
  sejong::pretty_print(vec1, std::cout, "R elbow Pitch link");

  visualization_msgs::Marker cyl = createCylinder(vec1, vec);
  sejong::Quaternion joint1Orien;
  valkyrie_collision_checker->robot_model->getOrientation(m_q, SJLinkID::LK_rightShoulderRollLink, joint1Orien);
  Eigen::Matrix3d rotation = joint1Orien.normalized().toRotationMatrix();
  Eigen::Matrix3d extraRot; // needed because arms are based around y axis
  // rotate 90 degrees about x axis
  extraRot << 1, 0, 0,
              0, 0, -1,
              0, 1, 0;

  rotation = rotation*extraRot;
  sejong::Quaternion newOrientation(rotation);
  sejong::pretty_print(newOrientation, std::cout, "Cyl:");
  cyl.pose.orientation.x = newOrientation.x();
  cyl.pose.orientation.y = newOrientation.y();
  cyl.pose.orientation.z = newOrientation.z();
  cyl.pose.orientation.w = newOrientation.w();

  markerVector.push_back(cyl);







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
    // printCollisions(collisionContacts);
   
    // std::cout << "Debug3" << std::endl;
    r.sleep();
  }


  delete colliderObstacle;
  delete valkyrie_collision_checker;
  delete boxString;
  return -1;
}


int main(int argc, char** argv) {

  // Initialize a ROS node
  ros::init(argc, argv, "fcl_tests");
  ros::NodeHandle n;

  // temporary declaration
  colliderObstacle = generateObstacle(sejong::Vect3(100, 100, 100), .25);
  boxString = new std::string("box");

  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  ros::Subscriber sub = n.subscribe("interactiveMarker", 10, colliderCallback);
  ros::Rate r(NODE_RATE);

  sejong::Vector m_q; m_q.resize(NUM_Q); m_q.setZero();
  sejong::Vector m_qdot; m_qdot.resize(NUM_QDOT); m_qdot.setZero();

  m_q[NUM_QDOT] = 1.0;

  RobotModel* robot_model= RobotModel::GetRobotModel();
  robot_model->UpdateModel(m_q, m_qdot);  


  sejong::Vect3 vec;
  sejong::Vect3 vec1;
  
  robot_model->getPosition(m_q, SJLinkID::LK_rightShoulderPitchLink, vec);
  sejong::pretty_print(vec, std::cout, "R Shoulder pitch link");
  robot_model->getPosition(m_q, SJLinkID::LK_rightElbowPitchLink, vec1);
  sejong::pretty_print(vec1, std::cout, "R elbow Pitch link");


  std::vector<visualization_msgs::Marker> markerVector;

  // cylindrical marker
  visualization_msgs::Marker cyl = createCylinder(vec1, vec);
  sejong::Quaternion joint1Orien;
  robot_model->getOrientation(m_q, SJLinkID::LK_rightShoulderRollLink, joint1Orien);
  Eigen::Matrix3d rotation = joint1Orien.normalized().toRotationMatrix();
  Eigen::Matrix3d extraRot; // needed because arms are based around y axis
  // rotate 90 degrees about x axis
  extraRot << 1, 0, 0,
              0, 0, -1,
              0, 1, 0;

  rotation = rotation*extraRot;
  sejong::Quaternion newOrientation(rotation);
  sejong::pretty_print(newOrientation, std::cout, "Cyl:");
  cyl.pose.orientation.x = newOrientation.x();
  cyl.pose.orientation.y = newOrientation.y();
  cyl.pose.orientation.z = newOrientation.z();
  cyl.pose.orientation.w = newOrientation.w();

  markerVector.push_back(cyl);



  // Set up array markers message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.resize(markerVector.size());

  // add markers to message
  for(int i=0; i<markerVector.size(); i++) {
    markerArray.markers[i] = markerVector[i];
  }





  // generate FCL object
  double dist = calcDistance(vec, vec1);

  fcl::Cylinder<double> fcl_cyl(.08, dist);
  fcl::Transform3<double> collisionTran;
  sejong::Vect3 position_final = calcMidpoint(vec, vec1);

  position_final[2] -= .12; // fixing for the rotation
  collisionTran.translation() = position_final;
  std::shared_ptr<fcl::CollisionGeometry<double>> fcl_colGeo(&fcl_cyl);
  fcl::CollisionObject<double> fcl_colObj(fcl_colGeo, collisionTran);
  fcl_colObj.setQuatRotation(newOrientation);



  fcl::BroadPhaseCollisionManager<double> *modelCollider = new fcl::SaPCollisionManager<double>();
  // Create collision objects to place into a model
  std::vector<fcl::CollisionObject<double>*> env;
  env.push_back(&fcl_colObj);


  // add the collision object to the model collider
  modelCollider->registerObjects(env);

  // Initialize the manager
  modelCollider->setup();




  std::cout << "ROS Loop start" << std::endl;
  while(ros::ok()) {
    ros::spinOnce();
    // publish marker
    marker_pub.publish(markerArray); 
    fcl::test::CollisionData<double> *colData = new fcl::test::CollisionData<double>();
    modelCollider->collide(colliderObstacle, colData, fcl::test::defaultCollisionFunction);
    std::cout << "There are " << colData->result.numContacts() << " collisions, yay!" << std::endl;
    // std::vector<fcl::Contact<double>> collisionContacts = valkyrie_collision_checker->collideWith(colliderObstacle);
    r.sleep();
  }
}

