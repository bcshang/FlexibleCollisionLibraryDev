#ifndef __COLLISION_LINK_HPP
#define __COLLISION_LINK_HPP
/**
 * Description updated: April 24, 2018
 * 
 * This class is meant as an FCL abstraction to a joint.
 * 
 *     By providing the two link enumerations and a link type (leg, arm, torso, etc. which is defined in RCC_Common.hpp)
 * it will generate a collision object based on a passed in valkyrie joint configuration and a hard coded shape. Look at
 * RCC_Common.hpp for link size definitions.
 *
 *     I also provide a special function to generate a collision object based on passed in x,y,z sizes that I use for 
 * torso/head generation. 
 */



#include "RCC_Common.hpp"
#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include "Valkyrie_Definition.h"

#include "fcl/geometry/shape/shape_base.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/box.h"


#include "helpfulMath.hpp"
#include <iostream>

#ifdef SHOWMARKERS
#include "markerGeneration.hpp"
#endif

template<typename S>
class CollisionLink{
public:
    int link1; // SJLinkID's
    int link2;
    int linkType;
    int markerID;

    std::shared_ptr<fcl::ShapeBase<S> > collisionShape; // Shape
    fcl::Transform3<S> collisionTran;   // transformation
    std::shared_ptr<fcl::CollisionGeometry<S>> collisionGeo; // collision geometry
    fcl::CollisionObject<S>* collisionObj;   // collision object
    RobotModel* robot_model;

    #if SHOWMARKERS
    visualization_msgs::Marker jointMarker;
    #endif

    CollisionLink(int firstlink, int secondlink, int linkType);
    CollisionLink(int firstlink, int secondlink, int linkType, int markerID);
    ~CollisionLink();

    fcl::CollisionObject<S>* computeCollisionObject(sejong::Vector& robot_q);
    fcl::CollisionObject<S>* computeCollisionObject(sejong::Vector& robot_q, double x_size, double y_size, double z_size);
};

#include "CollisionLink-inl.hpp"

#endif
