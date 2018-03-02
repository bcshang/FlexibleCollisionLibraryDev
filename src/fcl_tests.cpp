// Standard C things
#include <iostream>
#include <vector>
#include <math.h>

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
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"

#include "test_fcl_utility.h"

// Valkyrie things
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include "Valkyrie_Definition.h"

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
  
  fcl::BVHModel<fcl::OBBRSS<double>> *colliderModel = new fcl::BVHModel<fcl::OBBRSS<double>>();
  
  fcl::Transform3<double> obstacleTransform = fcl::Transform3<double>(fcl::Translation3<double>(location));
  
  // generateBVHModel(*colliderModel, obstacleCube, obstacleTransform);
  
  fcl::CollisionObject<double>* colliderObject = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(obstacleCube), obstacleTransform);

  return colliderObject;
}

void colliderCallback(const geometry_msgs::Pose& msg) {
  delete colliderObstacle;
  colliderObstacle = generateObstacle(sejong::Vect3(msg.position.x, msg.position.y, msg.position.z), sizeBox);
}



int main(int argc, char** argv) {

  // Initialize a ROS node
  ros::init(argc, argv, "fcl_tests");
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  ros::Subscriber sub = n.subscribe("interactiveMarker", 10, colliderCallback);
  ros::Rate r(NODE_RATE);



  // Get Valkyrie information
  RobotModel* valkyrie_model = RobotModel::GetRobotModel();

  sejong::Vector m_q; m_q.resize(NUM_Q); m_q.setZero();
  sejong::Vector m_qdot; m_qdot.resize(NUM_QDOT); m_qdot.setZero();


  m_q[NUM_QDOT] = 1.0;

  valkyrie_model->UpdateModel(m_q, m_qdot);  

  sejong::Vect3 left_knee_pos;
  sejong::Vect3 left_foot_pos;
  
  valkyrie_model->getPosition(m_q, SJLinkID::LK_leftKneePitchLink, left_knee_pos);
  valkyrie_model->getPosition(m_q, SJLinkID::LK_leftCOP_Frame, left_foot_pos);

  // Print out the calculated location of the joints
  sejong::pretty_print(left_foot_pos, std::cout, "left_foot_pos");  
  sejong::pretty_print(left_knee_pos, std::cout, "left_knee_pos");  




  // Variable size array
  std::vector<visualization_msgs::Marker> markerVector;

  // create markers based on the calculated link positions
  //markerVector.push_back(createMarker(left_foot_pos));
  //markerVector.push_back(createMarker(left_knee_pos));
  markerVector.push_back(createCylinder(left_foot_pos, left_knee_pos));

  // Set up array markers message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.resize(markerVector.size());

  // add markers to message
  for(int i=0; i<markerVector.size(); i++) {
    markerArray.markers[i] = markerVector[i];
  }




  /* Start FCL Things */
  // Create FCL things
  double dist = calcDistance(left_foot_pos, left_knee_pos);
  std::cout << "Shin length: " << dist << std::endl;

  // Create a cylinder to mock the shin
  fcl::Cylinder<double> shinCyl(.10, dist);

  // Declare bounded volume heierarchy model with oriented bounding box + rectangular sphere swept
  fcl::BVHModel<fcl::OBBRSS<double>> *shinModel = new fcl::BVHModel<fcl::OBBRSS<double>>();

  // Not sure, but hopefully this places the cylinder where the shin should be
  fcl::Transform3<double> shinTransform = fcl::Transform3<double>(fcl::Translation3<double>(left_foot_pos));

  // Populate shin model with the declared cylinder
  // generateBVHModel(*shinModel, shinCyl, shinTransform, 16, 16);

  // temporary declaration
  colliderObstacle = generateObstacle(sejong::Vect3(100, 100, 100), .25);
  
  // Declare a sweep and prune collision manager (mentioned in the research paper specifically for ROS?)
  fcl::BroadPhaseCollisionManager<double> *modelCollider = new fcl::SaPCollisionManager<double>();


  // Create collision objects to place into a model
  std::vector<fcl::CollisionObject<double>*> env;
  env.push_back(new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(&shinCyl), shinTransform));


  // add the collision object to the model collider
  modelCollider->registerObjects(env);

  // Initialize the manager
  modelCollider->setup();

  // returns information about our collision
  

  // Run the entire model against the collision object to figure out if there is a collision
  // Collision data will return in the colData variable
 

 

  while(ros::ok()) {

    // Publish the marker array
    marker_pub.publish(markerArray); 
    
    ros::spinOnce();
    fcl::test::CollisionData<double> *colData = new fcl::test::CollisionData<double>();
    modelCollider->collide(colliderObstacle, colData, fcl::test::defaultCollisionFunction);
    std::cout << "There are " << colData->result.numContacts() << " collisions, yay!" << std::endl;
    delete colData;
    r.sleep();
  }

  return -1;
}



