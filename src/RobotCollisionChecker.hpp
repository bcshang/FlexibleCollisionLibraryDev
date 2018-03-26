#ifndef __ROBOTCOLLISIONCHECKER_HPP
#define __ROBOTCOLLISIONCHECKER_HPP

#include <vector>

#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include "Valkyrie_Definition.h"

#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/narrowphase/collision_result.h"

#include "helpfulMath.hpp"

template <typename S>
class RobotCollisionChecker{
public:
  RobotCollisionChecker();

  RobotCollisionChecker(sejong::Vector& m_q, sejong::Vector& m_qdot);
  
  ~RobotCollisionChecker();

  RobotModel* robot_model;
  sejong::Vector* robot_q;
  sejong::Vector* robot_qdot;
  fcl::BroadPhaseCollisionManager<S> *robotCollisionModel; 

  /**
   * Regenerate BroadPhaseCollision Manager to run collisions against objects
   * Will use robot_q and robot_qdot to calculate joint positions and generate collision objects
   */
  void generateRobotCollisionModel();


  std::vector<fcl::Contact<S>> collideWith(fcl::CollisionObject<S>* obj);
};

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

  // Declare bounded volume heierarchy model with oriented bounding box + rectangular sphere swept
  //fcl::BVHModel<fcl::OBBRSS<S>> *shinModel = new fcl::BVHModel<fcl::OBBRSS<S>>();

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

template <typename S>
std::vector<fcl::Contact<S>> RobotCollisionChecker<S>::collideWith(fcl::CollisionObject<S>* obj){
    fcl::test::CollisionData<double> *colData = new fcl::test::CollisionData<double>();
    robotCollisionModel->collide(obj, colData, fcl::test::defaultCollisionFunction);
    
    std::vector<fcl::Contact<S>> contacts;
    colData->result.getContacts(contacts);
    
    delete colData;

    return contacts;
}





#endif