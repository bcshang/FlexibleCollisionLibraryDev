// Standard C things
#include <iostream>
#include <vector>
#include <math.h>

// ROS things
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// FCL things
#include "fcl/broadphase/broadphase_SaP.h"

#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"

#include "test_fcl_utility.h"

// Valkyrie things
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include "Valkyrie_Definition.h"



#define NODE_RATE 15

/**
 * Creates a ROS visualization marker
 * @param x coordinate of marker
 * @param y coordinate of marker
 * @param z coordinate of marker
 */
visualization_msgs::Marker createMarker(double x_pos, double y_pos, double z_pos) {
  visualization_msgs::Marker marker;
  static int markernum = 0;

  int shape = visualization_msgs::Marker::CUBE;


  marker.header.frame_id = "/val_robot/pelvis";
  marker.header.stamp = ros::Time::now();

  marker.ns = "basic_shapes";
  marker.id = markernum;
  markernum++;

  // Set the marker shape
  marker.type = shape;

  // Set the marker action
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker
  marker.pose.position.x = x_pos;
  marker.pose.position.y = y_pos;
  marker.pose.position.z = z_pos;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();


  return marker;
}



/**
 * Convenience wrapper for markers
 * @param  pos position of the marker
 * @return     ROS Marker message
 */
visualization_msgs::Marker createMarker(sejong::Vect3 pos) {
  return createMarker(pos[0], pos[1], pos[2]);
}





/**
 * Returns the distance between two points
 * @param  point1 first point
 * @param  point2 second point
 * @return        distance between points
 */
double calcDistance(sejong::Vect3 point1, sejong::Vect3 point2) {
  double distance1 = pow((point1[0] - point2[0]), 2);
  double distance2 = pow((point1[1] - point2[1]), 2);
  double distance3 = pow((point1[2] - point2[2]), 2);

  return sqrt(distance1 + distance2 + distance3);

}



template<class BV>
fcl::BVHModel<BV>* doSomething(void) {
  //std::shared_ptr<fcl::BVHModel<BV> > model(new fcl::BVHModel<BV>);
  
  fcl::BVHModel<BV>* model = new fcl::BVHModel<BV>;

  //model->beginModel();
  // Add crap to the model

  // for loop to add vertexes
  // may need to add vertexes for each individual section and make them submodels
  
  // Finshed adding
  //model->endModel();

  // calculate bounding box
  //model->computeLocalAABB();
  return model;
}

// auto shapeMesh = static_cast<const MeshShape*>(shape.get());
// const Eigen::Vector3d& scale = shapeMesh->getScale();
// auto aiScene = shapeMesh->getMesh();


int main(int argc, char** argv) {

  // Initialize a ROS node
  ros::init(argc, argv, "fcl_tests");
  ros::NodeHandle n;

  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
  ros::Rate r(NODE_RATE);



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
  markerVector.push_back(createMarker(left_foot_pos));
  markerVector.push_back(createMarker(left_knee_pos));

  // Set up array markers message
  visualization_msgs::MarkerArray markerArray;
  markerArray.markers.resize(markerVector.size());

  // add markers to message
  for(int i=0; i<markerVector.size(); i++) {
    markerArray.markers[i] = markerVector[i];
  }


  // Create FCL things
  double dist = calcDistance(left_foot_pos, left_knee_pos);
  std::cout << "Shin length: " << dist << std::endl;

  // Create a cylinder to mock the shin
  fcl::Cylinder<double> shinCyl(.10, dist);

  // Declare bounded volume heierarchy model
  fcl::BVHModel<fcl::OBBRSS<double>> *shinModel = new fcl::BVHModel<fcl::OBBRSS<double>>();

  // Not sure, but hopefully this places the cylinder where the shin should be
  fcl::Transform3<double> shinTransform = fcl::Transform3<double>(fcl::Translation3<double>(left_foot_pos));

  // Populate shin model with the declared cylinder
  generateBVHModel(*shinModel, shinCyl, shinTransform, 16, 16);


  // Random testing cylinder
  fcl::Cylinder<double> colliderCyl(.10, 1);
  fcl::BVHModel<fcl::OBBRSS<double>> *colliderModel = new fcl::BVHModel<fcl::OBBRSS<double>>();
  generateBVHModel(*colliderModel, colliderCyl, shinTransform, 16, 16);
  fcl::Transform3<double> stdTransform = fcl::Transform3<double>(fcl::Translation3<double>(fcl::Vector3<double>(0,0,0)));
  fcl::CollisionObject<double>* colliderCylObject = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(&colliderCyl), shinTransform);
  // fcl::CollisionObject<double>* colliderCylObject = new fcl::CollisionObject<double>(std::shared_ptr<fcl::CollisionGeometry<double>>(&colliderCyl), stdTransform);

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
  fcl::test::CollisionData<double> *colData = new fcl::test::CollisionData<double>();

  // Run the entire model against the collision object to figure out if there is a collision
  // Collision data will return in the colData variable
  modelCollider->collide(colliderCylObject, colData, fcl::test::defaultCollisionFunction);

  std::cout << "There are " << colData->result.numContacts() << " collisions, yay!" << std::endl;

  while(ros::ok()) {
    // Publish the marker array
    marker_pub.publish(markerArray);
    r.sleep();
  }


  doSomething<fcl::AABB<double>>();

  return -1;
}



