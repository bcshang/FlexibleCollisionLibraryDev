/**
 * For why I do this wierdness with an inline hpp inside of another hpp, see:
 * 1. https://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file
 * 2. How FCL decided to deal with this templating compiling annoyances
 */

#ifndef __ROBOT_COLLISION_CHECKER_INL_HPP
#define __ROBOT_COLLISION_CHECKER_INL_HPP

#include "RobotCollisionChecker.hpp"

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
  
  robot_model = RobotModel::GetRobotModel();  
  robot_q = new sejong::Vector(m_q);
  robot_qdot = new sejong::Vector(m_qdot);

  robot_model->UpdateModel(*robot_q, *robot_qdot);  

   // Declare a sweep and prune collision manager (mentioned in the research paper specifically for ROS?)
  robotCollisionModel = new fcl::SaPCollisionManager<S>();
  robotCollisionModel->setup();
}

/**
 * Destructor
 */
template <typename S>
RobotCollisionChecker<S>::~RobotCollisionChecker(){
  delete robot_q; 
  delete robot_qdot;
}

/**
 * Generates BroadPhaseCollision Manager to run collisions against objects
 */
template <typename S>
void RobotCollisionChecker<S>::generateRobotCollisionModel() {

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

  // This places the cylinder where the shin should be
  // and if it doesn't, we use the same style of reference for all objects so it doesn't matter
  fcl::Transform3<S> *shinTransform = new fcl::Transform3<S>(fcl::Translation3<S>(*left_foot_pos));
  
  
 


  // Create collision objects to place into a model
  std::vector<fcl::CollisionObject<S>*> env;

  std::string *s = new std::string("leftShin");

  // name the joint
  std::shared_ptr<fcl::CollisionGeometry<S>> shinGeom(shinCyl);
  shinGeom->setUserData(s);

  fcl::CollisionObject<S>* linkCollisionObject = new fcl::CollisionObject<S>(shinGeom, *shinTransform);
  linkCollisionObject->setUserData(s);

  env.push_back(linkCollisionObject);

  robotCollisionModel->clear();

  // add the collision object to the model collider
  robotCollisionModel->registerObjects(env);
  
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
