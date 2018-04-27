/**
 * For why I do this wierdness with an inline hpp inside of another hpp, see:
 * 1. https://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file
 * 2. How FCL decided to deal with this templating compiling annoyances
 */

#ifndef __ROBOT_COLLISION_CHECKER_INL_HPP
#define __ROBOT_COLLISION_CHECKER_INL_HPP

#include "RobotCollisionChecker.hpp"
#include <iostream>

/**
 * Actually use this constructor please
 */
template <typename S>
RobotCollisionChecker<S>::RobotCollisionChecker(sejong::Vector m_q, sejong::Vector m_qdot){
  markerNumber = 0;
  // Get the robot model
  robot_model = RobotModel::GetRobotModel();

  // Setup class variables  
  robot_q = m_q;
  robot_qdot = m_qdot;

  // update model
  robot_model->UpdateModel(robot_q, robot_qdot);  

   // Declare a sweep and prune collision manager (mentioned in the research paper specifically for ROS?)
  robotCollisionModel = new fcl::SaPCollisionManager<S>();
  robotCollisionModel->setup();

  // Initialize joint lists
  // Arms are missing hand joints
  rightArmJoints.push_back(SJLinkID::LK_rightShoulderRollLink);
  rightArmJoints.push_back(SJLinkID::LK_rightElbowPitchLink);

  leftArmJoints.push_back(SJLinkID::LK_leftShoulderRollLink);
  leftArmJoints.push_back(SJLinkID::LK_leftElbowPitchLink);

  rightLegJoints.push_back(SJLinkID::LK_rightCOP_Frame);
  rightLegJoints.push_back(SJLinkID::LK_rightKneePitchLink);
  rightLegJoints.push_back(SJLinkID::LK_rightHipYawLink);

  leftLegJoints.push_back(SJLinkID::LK_leftCOP_Frame);
  leftLegJoints.push_back(SJLinkID::LK_leftKneePitchLink);
  leftLegJoints.push_back(SJLinkID::LK_leftHipYawLink);

  torsoJoints.push_back(SJLinkID::LK_torso);
  torsoJoints.push_back(SJLinkID::LK_neckYawLink);
  torsoJoints.push_back(SJLinkID::LK_upperNeckPitchLink);

  // Populate all joints as collision link objects (class I defined)
  for(int i=0; i<rightArmJoints.size()-1; i++){
    collisionLinks.push_back(CollisionLink<S>(rightArmJoints[i], rightArmJoints[i+1], CLT_appendage_right_arm, markerNumber++));
  }
  for(int i=0; i<leftArmJoints.size()-1; i++){
    collisionLinks.push_back(CollisionLink<S>(leftArmJoints[i], leftArmJoints[i+1], CLT_appendage_left_arm, markerNumber++));
  }
  for(int i=0; i<rightLegJoints.size()-1; i++){
    collisionLinks.push_back(CollisionLink<S>(rightLegJoints[i], rightLegJoints[i+1], CLT_appendage_leg, markerNumber++));
  }
  for(int i=0; i<leftLegJoints.size()-1; i++){
    collisionLinks.push_back(CollisionLink<S>(leftLegJoints[i], leftLegJoints[i+1], CLT_appendage_leg, markerNumber++));
  }

  // Add the torso
  collisionLinks.push_back(CollisionLink<S>(SJLinkID::LK_torso, LK_neckYawLink, CLT_torso, markerNumber++));
}

/**
 * Destructor
 */
template <typename S>
RobotCollisionChecker<S>::~RobotCollisionChecker(){
  delete robotCollisionModel;
}



// Updater
template <typename S>
void RobotCollisionChecker<S>::updateQ(sejong::Vector m_q) {
  robot_q = m_q;
  robot_model->UpdateModel(robot_q, robot_qdot);  
}


// Updater
template <typename S>
void RobotCollisionChecker<S>::updateQ_Dot(sejong::Vector m_qdot){
  robot_qdot = m_qdot;
  robot_model->UpdateModel(robot_q, robot_qdot);  
}






/**
 * Generates BroadPhaseCollision Manager to run collisions against objects
 */
template <typename S>
void RobotCollisionChecker<S>::generateRobotCollisionModel() {
  robot_env.clear();
  // This will also delete all collision objects so deletion within the collisionlink class is not necessary
  robotCollisionModel->clear();


  // Collision environment is a class variable
  for(int i=0; i<collisionLinks.size(); i++){
    // Only add arms and legs. Torso and head must be done specially
    if(collisionLinks[i].linkType == collisionLinkType::CLT_appendage_left_arm || 
      collisionLinks[i].linkType == collisionLinkType::CLT_appendage_right_arm ||
      collisionLinks[i].linkType == collisionLinkType::CLT_appendage_leg)
        robot_env.push_back(collisionLinks[i].computeCollisionObject(robot_q));
  }

  // Torso must be done specially and really jankly
  // Start with the actual torso
  sejong::Vect3 joint1Pos;
  sejong::Vect3 joint2Pos;
  // get width
  robot_model->getPosition(robot_q, SJLinkID::LK_rightShoulderPitchLink, joint1Pos);
  robot_model->getPosition(robot_q, SJLinkID::LK_leftShoulderPitchLink, joint2Pos);
  double torsoWidth = calcDistance(joint1Pos, joint2Pos);
  
  // get hips to calculate height
  sejong::Vect3 temphip1;
  sejong::Vect3 temphip2;
  robot_model->getPosition(robot_q, SJLinkID::LK_leftHipYawLink, temphip1);
  robot_model->getPosition(robot_q, SJLinkID::LK_rightHipYawLink, temphip2);
  joint1Pos = calcMidpoint(temphip1, temphip2);

  // get the height of the object
  robot_model->getPosition(robot_q, SJLinkID::LK_neckYawLink, joint2Pos);
  double torsoHeight = calcDistance(joint1Pos, joint2Pos);


  robot_env.push_back(collisionLinks[collisionLinks.size()-1].computeCollisionObject(robot_q, torsoDepth, torsoWidth, torsoHeight));

  // add the collision object to the model collider
  robotCollisionModel->registerObjects(robot_env);
  
  // Initialize the manager
  robotCollisionModel->update();
}


/**
 * Grabs all joint markers from the joints for RVIZ visualization
 * @return visualization_msgs::Marker for all joints in a vector
 */
template <typename S>
std::vector<visualization_msgs::Marker> RobotCollisionChecker<S>::generateMarkers(void){
  // Variable size array
  std::vector<visualization_msgs::Marker> markerVector;
  for(int i=0; i< collisionLinks.size(); i++){
    markerVector.push_back(collisionLinks[i].jointMarker);
    // std::cout << markerVector[i] << std::endl << std::endl;
  }
  return markerVector;
}


/**
 * Main colliding function
 * 
 * @param obj       Collision Object to see if the robot is colliding with
 * @return    Result of the collision, documentation at: http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/structfcl_1_1CollisionResult.html
 *            
 *            In my experience, the first object will be obj and the second will be a valkyrie link
 */
template <typename S>
fcl::CollisionResult<S> RobotCollisionChecker<S>::collideWith(fcl::CollisionObject<S>* obj){
  // Run FCL's collide function
  fcl::test::CollisionData<double> colData;
  colData.request.num_max_contacts = 5; // Maximum number of contacts that will be returned by the function
  colData.request.enable_contact = true; // Tell the request to generate things like penetration depth and location

  robotCollisionModel->collide(obj, &colData, fcl::test::defaultCollisionFunction);
  
  // Grab all contact data
  // std::vector<fcl::Contact<S>> contacts;
  // colData.result.getContacts(contacts);

  return colData.result;
}


// self collision
template <typename S>
fcl::CollisionResult<S> RobotCollisionChecker<S>::collideSelf(){
  fcl::test::CollisionData<double> colData;
  colData.request.num_max_contacts = 5; // Maximum number of contacts that will be returned by the function
  colData.request.enable_contact = true;
  robotCollisionModel->collide(&colData, fcl::test::defaultCollisionFunction);

  // std::vector<fcl::Contact<S>> contacts;
  // colData.result.getContacts(contacts);


  return colData.result;
}

/**
 * Calculates the minimum distance to a collision object
 * @param obj the object to check distance to
 * @return the minimum distance to the object (can be negative)
 */
template<typename S>
double RobotCollisionChecker<S>::distanceTo(fcl::CollisionObject<S>* obj){
  fcl::test::DistanceData<double> distData;
  distData.request.enable_signed_distance = true;
  distData.request.enable_nearest_points = true;
  distData.request.gjk_solver_type = fcl::GST_LIBCCD;
  
  robotCollisionModel->distance(obj, &distData, fcl::test::defaultDistanceFunction);


  return distData.result.min_distance;

}


#endif
