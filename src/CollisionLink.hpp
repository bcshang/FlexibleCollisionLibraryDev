#ifndef __COLLISION_LINK_HPP
#define __COLLISION_LINK_HPP

#include <Utils/wrap_eigen.hpp>
#include <Utils/utilities.hpp>
#include <Valkyrie/Robot_Model/RobotModel.hpp>
#include "Valkyrie_Definition.h"

#include "fcl/geometry/shape/shape_base.h"
#include "fcl/geometry/shape/cylinder.h"
#include "fcl/geometry/shape/box.h"

#include "helpfulMath.hpp"

enum collisionLinkType{
    CLT_appendage_arm = 0,
    CLT_appendage_leg,
    CLT_torso
};

#define armWidth .08
#define legWidth .10
#define torsoWidth .5 // TODO No idea what this value actually is

template<typename S>
class CollisionLink{
public:
    int link1; // SJLinkID's
    int link2;
    int linkType;

    std::shared_ptr<fcl::ShapeBase<S> > collisionShape; // Shape
    fcl::Transform3<S> collisionTran;   // transformation
    std::shared_ptr<fcl::CollisionGeometry<S>> collisionGeo; // collision geometry
    fcl::CollisionObject<S>* collisionObj;   // collision object
    RobotModel* robot_model;

    CollisionLink();
    CollisionLink(int firstlink, int secondlink, int linkType);
    ~CollisionLink();

    fcl::CollisionObject<S>* computeCollisionObject(sejong::Vector& robot_q, int linkType);
};

#include "CollisionLink-inl.hpp"

#endif
