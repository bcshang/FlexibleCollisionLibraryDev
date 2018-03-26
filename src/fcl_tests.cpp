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
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "fcl/narrowphase/collision_result.h"

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


template <typename S>
class CollisionChecker{
public:
  CollisionChecker();

  CollisionChecker(sejong::Vector& m_q, sejong::Vector& m_qdot);
  
  ~CollisionChecker();

  RobotModel* robot_model;
  sejong::Vector* robot_q;
  sejong::Vector* robot_qdot;
  fcl::BroadPhaseCollisionManager<S> *robotCollisionModel; 

  void generateRobotCollisionModel();

};

template <typename S>
CollisionChecker<S>::CollisionChecker(){};

template <typename S>
CollisionChecker<S>::CollisionChecker(sejong::Vector& m_q, sejong::Vector& m_qdot){
  
  robot_model = RobotModel::GetRobotModel();  
  robot_q = new sejong::Vector(m_q);
  robot_qdot = new sejong::Vector(m_qdot);

  robot_model->UpdateModel(*robot_q, *robot_qdot);  

}

template <typename S>
CollisionChecker<S>::~CollisionChecker(){
  delete robot_q; 
  delete robot_qdot;
}

template <typename S>
void CollisionChecker<S>::generateRobotCollisionModel() {

  sejong::Vect3 *left_knee_pos = new sejong::Vect3();
  sejong::Vect3 *left_foot_pos = new sejong::Vect3();
  
  // Shin
  robot_model->getPosition(*robot_q, SJLinkID::LK_leftKneePitchLink, *left_knee_pos);
  robot_model->getPosition(*robot_q, SJLinkID::LK_leftCOP_Frame, *left_foot_pos);

  // Print out the calculated location of the joints
  //sejong::pretty_print(left_foot_pos, std::cout, "left_foot_pos");  
  //sejong::pretty_print(left_knee_pos, std::cout, "left_knee_pos");  


  sejong::Quaternion leftKneePitchQuat;
  robot_model->getOrientation(*robot_q, SJLinkID::LK_leftKneePitchLink, leftKneePitchQuat);
  // sejong::pretty_print(leftKneePitchQuat, std::cout, "Left Knee Pitch Quaternion");

  double dist = calcDistance(*left_foot_pos, *left_knee_pos);
  
  // Create a cylinder to mock the shin
  fcl::Cylinder<S> *shinCyl = new fcl::Cylinder<S>(.10, dist);

  // Declare bounded volume heierarchy model with oriented bounding box + rectangular sphere swept
  fcl::BVHModel<fcl::OBBRSS<S>> *shinModel = new fcl::BVHModel<fcl::OBBRSS<S>>();

  // Not sure, but hopefully this places the cylinder where the shin should be
  fcl::Transform3<S> *shinTransform = new fcl::Transform3<S>(fcl::Translation3<S>(*left_foot_pos));

  // Populate shin model with the declared cylinder
  // generateBVHModel(*shinModel, shinCyl, shinTransform, 16, 16);

  
  
  // Declare a sweep and prune collision manager (mentioned in the research paper specifically for ROS?)
  robotCollisionModel = new fcl::SaPCollisionManager<S>();


  // Create collision objects to place into a model
  std::vector<fcl::CollisionObject<S>*> env;

  std::string *s = new std::string("leftShin");

  // name the joint
  std::shared_ptr<fcl::CollisionGeometry<S>> shinGeom(shinCyl);
  shinGeom->setUserData(s);

  fcl::CollisionObject<S>* linkCollisionObject = new fcl::CollisionObject<S>(shinGeom, *shinTransform);
  linkCollisionObject->setUserData(s);

  env.push_back(linkCollisionObject);


  // add the collision object to the model collider
  robotCollisionModel->registerObjects(env);

  // Initialize the manager
  robotCollisionModel->setup();

}










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
  // need to fix for how fcl vs ros denotes markers
  colliderObstacle = generateObstacle(sejong::Vect3(msg.position.x, msg.position.y, msg.position.z-sizeBox/2), sizeBox);
}



int main(int argc, char** argv) {

  // Initialize a ROS node
  ros::init(argc, argv, "fcl_tests");
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  ros::Subscriber sub = n.subscribe("interactiveMarker", 10, colliderCallback);
  ros::Rate r(NODE_RATE);



  // Get Valkyrie information
  sejong::Vector m_q; m_q.resize(NUM_Q); m_q.setZero();
  sejong::Vector m_qdot; m_qdot.resize(NUM_QDOT); m_qdot.setZero();

  m_q[NUM_QDOT] = 1.0;

  CollisionChecker<double> *valkyrie_collision_checker = new CollisionChecker<double>(m_q, m_qdot);
  valkyrie_collision_checker->generateRobotCollisionModel();


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


  
  // temporary declaration
  colliderObstacle = generateObstacle(sejong::Vect3(100, 100, 100), .25);
 

 

  while(ros::ok()) {

    // Publish the marker array
    marker_pub.publish(markerArray); 
    
    ros::spinOnce();
    fcl::test::CollisionData<double> *colData = new fcl::test::CollisionData<double>();
    valkyrie_collision_checker->robotCollisionModel->collide(colliderObstacle, colData, fcl::test::defaultCollisionFunction);
    
    std::vector<fcl::Contact<double>> collisionContacts;
    colData->result.getContacts(collisionContacts);
    if(collisionContacts.size() > 0) {
      fcl::Contact<double> con = collisionContacts[0];
      std::cout << con.o1->getUserData() << std::endl;
      std::cout << *((std::string*)con.o2->getUserData()) << std::endl;
      std::cout << "Blarg" << std::endl;
    }

    //if(colData->result.isCollision())
      //std::cout << "Colliding with: " << colData->result.getContact(0).o1->getUserData() << " collisions, yay!" << std::endl;
    

    delete colData;
    r.sleep();
  }

  return -1;
}



