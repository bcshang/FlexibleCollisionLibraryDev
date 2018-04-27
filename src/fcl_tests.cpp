#include "RCC_Common.hpp"

// Standard C things
#include <iostream>
#include <vector>
#include <math.h>
#include <cstring>
#include <map>

// ROS things
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include "geometry_msgs/Pose.h"
#include <interactive_markers/interactive_marker_server.h>
#include <sensor_msgs/JointState.h>

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




std::map<std::string, int> jointName2ID;
sejong::Vector m_q; // Main robot_q
fcl::CollisionObject<double> *colliderObstacle;
std::string *boxString;
/**
 * Called by the callback to update the FCL object used to collide with Valkyrie
 * @param location position of the box
 * @param size size of the box
 * @return fcl collision object
 */
fcl::CollisionObject<double>* generateObstacle(sejong::Vect3 location, double size) {
  // declare a box
  fcl::Box<double> *obstacleCube = new fcl::Box<double>(size, size, size);
  // Declare a transform (just a translation for now)
  fcl::Transform3<double> obstacleTransform = fcl::Transform3<double>(fcl::Translation3<double>(location));
  
  // Declare a collision geometry
  std::shared_ptr<fcl::CollisionGeometry<double>> colGeom(obstacleCube);
  
  // Set the user data as simply a string
  colGeom->setUserData(boxString);


  fcl::CollisionObject<double>* colliderObject = new fcl::CollisionObject<double>(colGeom, obstacleTransform);
  colliderObject->setUserData(boxString);
  return colliderObject;
}



/**
 * ROS calls this whenever a new Pose message is sent from the Collider cube
 * @param msg message sent by another node to update the collision box
 */
void colliderCallback(const geometry_msgs::Pose& msg) {
  // delete colliderObstacle->getUserData();
  delete colliderObstacle;
  colliderObstacle = generateObstacle(sejong::Vect3(msg.position.x, msg.position.y, msg.position.z), sizeBox);
}


int jStatesUpdated = 0;
/**
 * Callback for the joint_state subscriber
 * Will read relevant joints from the information and update global m_q accordingly
 * @param robotJointStates joint_states as a ROS message from /val_robot/joint_states
 */
void jointStateCallback(const sensor_msgs::JointState robotJointStates) {
  for(int i=0; i<robotJointStates.name.size(); i++) {
    if (jointName2ID.find(robotJointStates.name[i]) != jointName2ID.end() ) {
      m_q[jointName2ID[robotJointStates.name[i]] + NUM_VIRTUAL] = robotJointStates.position[i];
    }
  }
  jStatesUpdated = 1;
}


/**
 * Prints collision information between the collision box and Valkyrie
 * @param result Collision result of valkyrie collision with colliderBox
 */
void printObjCollisions(fcl::CollisionResult<double> result) {
  std::vector<fcl::Contact<double>> collisionContacts;

  result.getContacts(collisionContacts);
  if(collisionContacts.size() > 0) {
    std::cout << "Number of obj collisions: " << collisionContacts.size() << std::endl;
    for(int i=0; i<collisionContacts.size(); i++) {
      fcl::Contact<double> con = collisionContacts[i];
      std::cout << "Collision Between: ";
      std::cout << *((std::string*)con.o1->getUserData()); // hard coded that the user data will be a string for the box
      std::cout << " and joint ";
      std::cout << ((CollisionLink<double>*)con.o2->getUserData())->link1 << "-" << ((CollisionLink<double>*)con.o2->getUserData())->link2 << std::endl;
      std::cout << "Penetration depth: " << con.penetration_depth << 
                  " \nwith vector from box to joint being: \n" << con.normal <<
                  " \nat position " << con.pos << std::endl;    
    }
  }
  else{
    std::cout << "No obj collisions" << std::endl;
  }
  std::cout << std::endl;
}


void printSelfCollisions(fcl::CollisionResult<double> result) {
  std::vector<fcl::Contact<double>> collisionContacts;

  result.getContacts(collisionContacts);
  if(collisionContacts.size() > 0) {
    std::cout << "Number of self collisions: " << collisionContacts.size() << std::endl;
    for(int i=0; i<collisionContacts.size(); i++) {
      fcl::Contact<double> con = collisionContacts[i];
      std::cout << "Collision between joint ";
      std::cout << ((CollisionLink<double>*)con.o1->getUserData())->link1 << "-" << ((CollisionLink<double>*)con.o1->getUserData())->link2 << std::endl;
      std::cout << " and joint ";
      std::cout << ((CollisionLink<double>*)con.o2->getUserData())->link1 << "-" << ((CollisionLink<double>*)con.o2->getUserData())->link2 << std::endl;
      

      std::cout << "Penetration depth: " << con.penetration_depth << 
                  " \nwith vector from link1 to link2 being: \n" << con.normal <<
                  " \nat position " << con.pos << std::endl << std::endl;    
    }
  }
  else{
    std::cout << "No self collisions" << std::endl;
  }
  std::cout << std::endl;
}


/**
 * Standard testing program
 */
void standardProgram(void) {
  boxString = new std::string("box");

  // Build joint table for updating joint states
  // Does not account for virtual joints
  jointName2ID.insert(std::make_pair("leftHipYaw", 0));
  jointName2ID.insert(std::make_pair("leftHipRoll", 1));
  jointName2ID.insert(std::make_pair("leftHipPitch", 2));
  jointName2ID.insert(std::make_pair("leftKneePitch", 3));
  jointName2ID.insert(std::make_pair("leftAnklePitch", 4));
  jointName2ID.insert(std::make_pair("leftAnkleRoll", 5));
  jointName2ID.insert(std::make_pair("rightHipYaw", 6));
  jointName2ID.insert(std::make_pair("rightHipRoll", 7));
  jointName2ID.insert(std::make_pair("rightHipPitch", 8));
  jointName2ID.insert(std::make_pair("rightKneePitch", 9));
  jointName2ID.insert(std::make_pair("rightAnklePitch", 10));
  jointName2ID.insert(std::make_pair("rightAnkleRoll", 11));
  jointName2ID.insert(std::make_pair("torsoYaw", 12));
  jointName2ID.insert(std::make_pair("torsoPitch", 13));
  jointName2ID.insert(std::make_pair("torsoRoll", 14));
  jointName2ID.insert(std::make_pair("leftShoulderPitch", 15));
  jointName2ID.insert(std::make_pair("leftShoulderRoll", 16));
  jointName2ID.insert(std::make_pair("leftShoulderYaw", 17));
  jointName2ID.insert(std::make_pair("leftElbowPitch", 18));
  jointName2ID.insert(std::make_pair("leftForearmYaw", 19));
  jointName2ID.insert(std::make_pair("lowerNeckPitch", 20));
  jointName2ID.insert(std::make_pair("neckYaw", 21));
  jointName2ID.insert(std::make_pair("upperNeckPitch", 22));
  jointName2ID.insert(std::make_pair("rightShoulderPitch", 23));
  jointName2ID.insert(std::make_pair("rightShoulderRoll", 24));
  jointName2ID.insert(std::make_pair("rightShoulderYaw", 25));
  jointName2ID.insert(std::make_pair("rightElbowPitch", 26));
  jointName2ID.insert(std::make_pair("rightForearmYaw", 27));

  // Setup Valkyrie information
  // This is done early because Valkyrie will sometimes lose its body otherwise
  m_q.resize(NUM_Q); m_q.setZero();
  sejong::Vector m_qdot; m_qdot.resize(NUM_QDOT); m_qdot.setZero();

  m_q[NUM_QDOT] = 1.0;



  // temporary declaration so the callback doesn't crash because it deletes when called
  // Also makes sure that the program doesn't crash if marker isn't launched
  colliderObstacle = generateObstacle(sejong::Vect3(100, 100, 100), .25);

  // Start ROS things
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10); // Publish markers to RVIZ
  ros::Subscriber sub = n.subscribe("interactiveMarker", 10, colliderCallback); // Update internal FCL object for the interactive marker
  ros::Subscriber sub2 = n.subscribe("/val_robot/joint_states", 10, jointStateCallback); // update internal variable for joint_q
  ros::Rate r(NODE_RATE);

  // Wait for a message from the robot model to update m_q
  while(!jStatesUpdated){
    ros::spinOnce();
  }

  // Start the collision model
  RobotCollisionChecker<double> *valkyrie_collision_checker = new RobotCollisionChecker<double>(m_q, m_qdot);
  std::cout << "Robot collision Checker generated" << std::endl;
  valkyrie_collision_checker->generateRobotCollisionModel();
  std::cout << "Valkyrie FCL Model Constructed" << std::endl;

  




  std::cout << "ROS Loop start" << std::endl;
  while(ros::ok()) {    
    ros::spinOnce();

    // Visualization markers for the FCL objects
    #if SHOWMARKERS
    std::vector<visualization_msgs::Marker> markerVector; 
    markerVector = valkyrie_collision_checker->generateMarkers();
    // Set up array markers message
    visualization_msgs::MarkerArray markerArray;
    markerArray.markers.resize(markerVector.size());

    // add markers to message
    for(int i=0; i<markerVector.size(); i++) {
      markerArray.markers[i] = markerVector[i];
    }
    marker_pub.publish(markerArray); 
    #endif
    
    // update valkyrie's model
    valkyrie_collision_checker->updateQ(m_q);
    valkyrie_collision_checker->generateRobotCollisionModel();

    // Collide with object
    fcl::CollisionResult<double> val_obj_collision_result = valkyrie_collision_checker->collideWith(colliderObstacle);
    double distance = valkyrie_collision_checker->distanceTo(colliderObstacle);
    
    // Collide with self
    fcl::CollisionResult<double> val_self_collision_result = valkyrie_collision_checker->collideSelf();

    #if DEBUG
    std::cout << "Distance to object is " << distance << std::endl;
    printObjCollisions(val_obj_collision_result);
    printSelfCollisions(val_self_collision_result);
    #endif
    
    r.sleep();
  }


  delete colliderObstacle;
  delete valkyrie_collision_checker;
  delete boxString;

}





void test(void) {

  // Initialize a ROS node
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
  m_q[NUM_VIRTUAL+SJJointID::rightShoulderRoll] = 0.5;

  RobotModel* robot_model= RobotModel::GetRobotModel();
  robot_model->UpdateModel(m_q, m_qdot);  


  sejong::Vect3 vec;
  sejong::Vect3 vec1;
  
  robot_model->getPosition(m_q, SJLinkID::LK_rightShoulderPitchLink, vec);
  sejong::pretty_print(vec, std::cout, "R Shoulder pitch link");
  robot_model->getPosition(m_q, SJLinkID::LK_rightElbowPitchLink, vec1);
  sejong::pretty_print(vec1, std::cout, "R elbow Pitch link");


  std::vector<visualization_msgs::Marker> markerVector;

  // cylindrical marker TODO Fix this
  visualization_msgs::Marker cyl;// = createCylinder(sejong::Vect3(0,0,-.5), sejong::Vect3(0,0,.5));
  sejong::Quaternion joint1Orien;
  robot_model->getOrientation(m_q, SJLinkID::LK_rightShoulderRollLink, joint1Orien);
  Eigen::Matrix3d rotation = joint1Orien.normalized().toRotationMatrix();
  Eigen::Matrix3d extraRot; // needed because arms are based around y axis
  // rotate 90 degrees about x axis
  extraRot << 1, 0, 0,
              0, 0, -1,
              0, 1, 0;

  rotation = extraRot*rotation;
  sejong::Quaternion newOrientation(rotation);
  sejong::pretty_print(newOrientation, std::cout, "Cyl:");
  cyl.pose.position.x += .5;
  cyl.pose.position.y += .5;
  cyl.pose.position.z += .5;
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

  fcl::Cylinder<double> fcl_cyl(.08, 1);
  fcl::Transform3<double> collisionTran;
  sejong::Vect3 position_final = sejong::Vect3(.5,.5,.5); //calcMidpoint(vec, vec1);

  // position_final[2] -= .12; // fixing for the rotation
  collisionTran.translation() = position_final;
  std::shared_ptr<fcl::CollisionGeometry<double>> fcl_colGeo(&fcl_cyl);
  fcl::CollisionObject<double> fcl_colObj(fcl_colGeo, collisionTran);
  fcl_colObj.setQuatRotation(newOrientation);

  fcl::BroadPhaseCollisionManager<double> *modelCollider = new fcl::SaPCollisionManager<double>();
  // Create collision objects to place into a model
  std::vector<fcl::CollisionObject<double>*> env;


  fcl::Cylinder<double> fcl_cyl2(.08, 1);
  fcl::Transform3<double> collisionTran2;
  sejong::Vect3 position_final2 = sejong::Vect3(100,.4,.4); //calcMidpoint(vec, vec1);

  // position_final[2] -= .12; // fixing for the rotation
  collisionTran2.translation() = position_final2;
  std::shared_ptr<fcl::CollisionGeometry<double>> fcl_colGeo2(&fcl_cyl);
  fcl::CollisionObject<double> fcl_colObj2(fcl_colGeo, collisionTran2);
  fcl_colObj2.setQuatRotation(newOrientation);





  // copied
  // Declare two spheres of radius 20 and 10
  fcl::Sphere<double> s1{20};
  fcl::Sphere<double> s2{10};

  // Create a 3x4 matrix (3 x 3+1)
  fcl::Transform3<double> tf1{fcl::Transform3<double>::Identity()};
  fcl::Transform3<double> tf2{fcl::Transform3<double>::Identity()};

  // Set up a distance request and fill in parameters(DistanceRequest has only a default constructor)
  fcl::DistanceRequest<double> request;
  request.enable_signed_distance = true;
  request.enable_nearest_points = true;
  request.gjk_solver_type = fcl::GST_LIBCCD;

  fcl::DistanceResult<double> result;

  bool res{false};
  // Expecting distance to be -5
  result.clear();
  tf2.translation() = fcl::Vector3<double>(23, 0, 0);
  res = distance(&s1, tf1, &s2, tf2, request, result);
  std::cout << "result.min_distance" << std::endl;
  std::cout << result.min_distance << std::endl;




  env.push_back(&fcl_colObj);



  // add the collision object to the model collider
  modelCollider->registerObjects(env);

  // Initialize the manager
  modelCollider->setup();


  // Random test
  fcl::DistanceRequest<double> distReq;
  fcl::DistanceResult<double> distRes;

  distance(&fcl_colObj, &fcl_colObj2, distReq, distRes);

  std::cout << "Distance is " << distRes.min_distance << std::endl;





  std::cout << "ROS Loop start" << std::endl;
  while(ros::ok()) {
    ros::spinOnce();
    // publish marker
    marker_pub.publish(markerArray); 
    fcl::test::CollisionData<double> *colData = new fcl::test::CollisionData<double>();
    modelCollider->collide(colliderObstacle, colData, fcl::test::defaultCollisionFunction);
    // std::cout << "There are " << colData->result.numContacts() << " collisions, yay!" << std::endl;
    // std::vector<fcl::Contact<double>> collisionContacts = valkyrie_collision_checker->collideWith(colliderObstacle);
    r.sleep();
  }
}




int main(int argc, char** argv) {
  // Initialize a ROS node
  ros::init(argc, argv, "fcl_tests");

  standardProgram();
  // test();
  return -1;
}



