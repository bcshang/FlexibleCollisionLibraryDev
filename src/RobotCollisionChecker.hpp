#ifndef __ROBOT_COLLISION_CHECKER_HPP
#define __ROBOT_COLLISION_CHECKER_HPP

#include <vector>

#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include "Valkyrie_Definition.h"

#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/box.h"
#include "fcl/narrowphase/collision_result.h"
#include "CollisionLink.hpp"
#include "helpfulMath.hpp"

template <typename S>
class RobotCollisionChecker{
public:

  std::vector<int> rightArmJoints;
  std::vector<int> leftArmJoints;
  std::vector<int> rightLegJoints;
  std::vector<int> leftLegJoints;
  std::vector<int> torsoJoints;

  RobotModel* robot_model;
  sejong::Vector* robot_q;
  sejong::Vector* robot_qdot;
  fcl::BroadPhaseCollisionManager<S> *robotCollisionModel; 
  std::vector<fcl::CollisionObject<S>*> robot_env;
  std::vector<CollisionLink<S>> collisionLinks;
  RobotCollisionChecker();

  RobotCollisionChecker(sejong::Vector& m_q, sejong::Vector& m_qdot);
  
  ~RobotCollisionChecker();


  /**
   * Recompute internal BroadPhaseCollisionManager to run collisions against objects
   * Will use robot_q and robot_qdot to calculate joint positions and generate collision objects
   */
  void generateRobotCollisionModel();

  /**
   * Main colliding function
   * 
   * @param obj       Collision Object to see if the robot is colliding with
   * @return    vector of all Contacts. To extract data, o1 and o2 are internal CollisionGeometry
   *            these internal classes have a function called getUserData which returns a pointer
   *            to whatever data you put in it
   */
  std::vector<fcl::Contact<S>> collideWith(fcl::CollisionObject<S>* obj);

  
};


/**
 * For why I do this wierdness with an inline hpp inside of another hpp, see:
 * 1. https://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file
 * 2. How FCL decided to deal with this templating compiling annoyances
 */
#include "RobotCollisionChecker-inl.hpp"


#endif