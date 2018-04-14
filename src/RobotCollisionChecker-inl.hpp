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
 * Default Constructor
 */
template <typename S>
RobotCollisionChecker<S>::RobotCollisionChecker(){};

/**
 * Actually use this constructor please
 */
template <typename S>
RobotCollisionChecker<S>::RobotCollisionChecker(sejong::Vector m_q, sejong::Vector m_qdot){
  
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
  rightArmJoints.push_back(SJLinkID::LK_rightElbowPitchLink);
  rightArmJoints.push_back(SJLinkID::LK_rightShoulderPitchLink);

  leftArmJoints.push_back(SJLinkID::LK_leftShoulderPitchLink);
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

  // Populate all joints as collision link objects
  for(int i=0; i<rightArmJoints.size()-1; i++){
    collisionLinks.push_back(CollisionLink<S>(rightArmJoints[i], rightArmJoints[i+1], CLT_appendage_arm));
  }
  for(int i=0; i<leftArmJoints.size()-1; i++){
    collisionLinks.push_back(CollisionLink<S>(leftArmJoints[i], leftArmJoints[i+1], CLT_appendage_arm));
  }
  for(int i=0; i<rightLegJoints.size()-1; i++){
    collisionLinks.push_back(CollisionLink<S>(rightLegJoints[i], rightLegJoints[i+1], CLT_appendage_leg));
  }
  for(int i=0; i<leftLegJoints.size()-1; i++){
    collisionLinks.push_back(CollisionLink<S>(leftLegJoints[i], leftLegJoints[i+1], CLT_appendage_leg));
  }
  // for(int i=0; i<torsoJoints.size()-1; i++){
  //   collisionLinks.push_back(CollisionLink<S>(torsoJoints[i], torsoJoints[i+1], CLT_torso));
  // }


}

/**
 * Destructor
 */
template <typename S>
RobotCollisionChecker<S>::~RobotCollisionChecker(){
  delete robotCollisionModel;
}

/**
 * Generates BroadPhaseCollision Manager to run collisions against objects
 */
template <typename S>
void RobotCollisionChecker<S>::generateRobotCollisionModel() {
  robot_env.clear();
  // Collision environment is a class variable
  for(int i=0; i<collisionLinks.size(); i++){
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
  
  // get hips to calculat height
  sejong::Vect3 temphip1;
  sejong::Vect3 temphip2;
  robot_model->getPosition(robot_q, SJLinkID::LK_leftHipYawLink, temphip1);
  robot_model->getPosition(robot_q, SJLinkID::LK_rightHipYawLink, temphip2);
  joint1Pos = calcMidpoint(temphip1, temphip2);

  // get the height of the object
  robot_model->getPosition(robot_q, SJLinkID::LK_neckYawLink, joint2Pos);
  double torsoHeight = calcDistance(joint1Pos, joint2Pos);

  // Actually add it to the model
  collisionLinks.push_back(CollisionLink<S>(SJLinkID::LK_torso, LK_neckYawLink, CLT_torso));
  robot_env.push_back(collisionLinks[collisionLinks.size()-1].computeCollisionObject(robot_q, torsoDepth, torsoWidth, torsoHeight));




  robotCollisionModel->clear();

  // add the collision object to the model collider
  robotCollisionModel->registerObjects(robot_env);
  
  // Initialize the manager
  robotCollisionModel->update();
}


/**
 * 
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
 * @return    vector of all Contacts. To extract data, o1 and o2 are internal CollisionGeometry
 *            these internal classes have a function called getUserData which returns a pointer
 *            to whatever data you put in it
 */
template <typename S>
std::vector<fcl::Contact<S>> RobotCollisionChecker<S>::collideWith(fcl::CollisionObject<S>* obj){
  // Run FCL's collide function
  fcl::test::CollisionData<double> colData;
  robotCollisionModel->collide(obj, &colData, fcl::test::defaultCollisionFunction);
  
  // Grab all contact data
  std::vector<fcl::Contact<S>> contacts;
  colData.result.getContacts(contacts);

  return contacts;
}


#endif
