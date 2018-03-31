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
RobotCollisionChecker<S>::RobotCollisionChecker(sejong::Vector& m_q, sejong::Vector& m_qdot){
  
  // Get the robot model
  robot_model = RobotModel::GetRobotModel();

  // Setup class variables  
  robot_q = new sejong::Vector(m_q);
  robot_qdot = new sejong::Vector(m_qdot);

  // update model
  robot_model->UpdateModel(*robot_q, *robot_qdot);  

   // Declare a sweep and prune collision manager (mentioned in the research paper specifically for ROS?)
  robotCollisionModel = new fcl::SaPCollisionManager<S>();
  robotCollisionModel->setup();

  // Declare all joints as collisionLink objects
  CollisionLink<S> link(SJLinkID::LK_leftCOP_Frame, SJLinkID::LK_leftKneePitchLink);
  collisionLinks.push_back(link);
}

/**
 * Destructor
 */
template <typename S>
RobotCollisionChecker<S>::~RobotCollisionChecker(){
  delete robot_q; 
  delete robot_qdot;
  delete robotCollisionModel;
}

/**
 * Generates BroadPhaseCollision Manager to run collisions against objects
 */
template <typename S>
void RobotCollisionChecker<S>::generateRobotCollisionModel() {  
  // Collision environment is a class variable
  robot_env.push_back(collisionLinks[0].computeCollisionObject(*robot_q, collisionLinkType::CLT_appendage_leg));
  robotCollisionModel->clear();

  // add the collision object to the model collider
  robotCollisionModel->registerObjects(robot_env);
  
  // Initialize the manager
  robotCollisionModel->update();
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
