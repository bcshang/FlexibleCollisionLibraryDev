#ifndef __ROBOT_COLLISION_CHECKER_HPP
#define __ROBOT_COLLISION_CHECKER_HPP
/**
 * Description updated: April 24, 2018
 * 
 * Main object to be created to check collisions between valkyrie and the world or valkyrie and itself
 *
 * 
 */


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


#include "markerGeneration.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
template <typename S>
class RobotCollisionChecker{
public:
  // vector of joint ID's for respective joints
  std::vector<int> rightArmJoints;
  std::vector<int> leftArmJoints;
  std::vector<int> rightLegJoints;
  std::vector<int> leftLegJoints;
  std::vector<int> torsoJoints;

  // Robot model
  RobotModel* robot_model;
  sejong::Vector robot_q;
  sejong::Vector robot_qdot;

  // FCL things
  fcl::BroadPhaseCollisionManager<S> *robotCollisionModel; 
  std::vector<fcl::CollisionObject<S>*> robot_env;

  // Vector of all joints in the robot
  std::vector<CollisionLink<S>> collisionLinks;

  int markerNumber; // visualization things

  // Main constructor
  // Will maintain internal copy of m_q and m_qdot
  RobotCollisionChecker(sejong::Vector m_q, sejong::Vector m_qdot);
  
  // main destructor
  ~RobotCollisionChecker();


  /**
   * Recompute internal BroadPhaseCollisionManager to run collisions against objects
   * Will use internal robot_q and robot_qdot to calculate joint positions and generate collision objects
   */
  void generateRobotCollisionModel();

   
  /**
   * Main colliding function
   * @param obj       Collision Object to see if the robot is colliding with
   * @return    Result of the collision, documentation at: http://gamma.cs.unc.edu/FCL/fcl_docs/webpage/generated/structfcl_1_1CollisionResult.html
   *            
   *            In my experience, the first object will be obj and the second will be a valkyrie link
   */
  fcl::CollisionResult<S> collideWith(fcl::CollisionObject<S>* obj);

  /**
   * Self collision function
   * @return result of the self collision, trimmed of links that are adjacent because of how we draw the objects
   *
   * In my experience, there is no consistency for what order the collisions are returned
   */
  fcl::CollisionResult<S> collideSelf(void);

  /**
   * Calculates the minimum distance to a collision object
   * @param obj the object to check distance to
   * @return the minimum distance to the object (can be negative)
   */
  double distanceTo(fcl::CollisionObject<S>* obj);

  /**
   * TODO This doesn't really do anything at this point
   * please view the actual function definition for how to use
   */
  void distanceSelf();

  // Update internal variables
  void updateQ(sejong::Vector m_q);
  void updateQ_Dot(sejong::Vector m_qdot);

  /**
   * Grabs all joint markers from the joints for RVIZ visualization
   * @return visualization_msgs::Marker for all joints in a vector
   */
  std::vector<visualization_msgs::Marker> generateMarkers(void);
};


/**
 * For why I do this wierdness with an inline hpp inside of another hpp, see:
 * 1. https://stackoverflow.com/questions/495021/why-can-templates-only-be-implemented-in-the-header-file
 * 2. How FCL decided to deal with this templating compiling annoyances
 */
#include "RobotCollisionChecker-inl.hpp"


#endif