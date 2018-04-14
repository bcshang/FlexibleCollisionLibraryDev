#ifndef __COLLISION_LINK_INL_HPP
#define __COLLISION_LINK_INL_HPP
#include "CollisionLink.hpp"

/**
 * Use this constructor
 * 
 */
template <typename S>
CollisionLink<S>::CollisionLink(int firstlink, int secondlink, int linkT)
    :  link1(firstlink), link2(secondlink), linkType(linkT)
{
    collisionObj = NULL;
    robot_model = RobotModel::GetRobotModel(); 
}

/**
 * Default Constructor
 */
template <typename S>
CollisionLink<S>::~CollisionLink(){}

/**
 * Main function to call
 */
template<typename S>
fcl::CollisionObject<S>* CollisionLink<S>::computeCollisionObject(sejong::Vector& robot_q) {
    // Get positions
    sejong::Vect3 joint1Pos;
    sejong::Vect3 joint2Pos;
    robot_model->getPosition(robot_q, link1, joint1Pos);
    robot_model->getPosition(robot_q, link2, joint2Pos);
    
    sejong::pretty_print(joint1Pos, std::cout, "Joint: " + std::to_string(link1) + " ");
    sejong::pretty_print(joint2Pos, std::cout, "Joint: " + std::to_string(link2) + " ");

    
    // Get orientations
    sejong::Quaternion joint1Orien;
    sejong::Quaternion joint2Orien;
    robot_model->getOrientation(robot_q, link1, joint1Orien);
    robot_model->getOrientation(robot_q, link2, joint2Orien);

    
    sejong::Vect3 fcl_position = calcMidpoint(joint1Pos, joint2Pos);
    // sejong::Vect3 fcl_position = joint1Pos;

    Eigen::Matrix3d rotation = joint1Orien.normalized().toRotationMatrix();
    //std::cout << "Rotation pre" << rotation << std::endl;
    if(linkType == collisionLinkType::CLT_appendage_arm) {
        Eigen::Matrix3d extraRot; // needed because arms are based around y axis
        // rotate 90 degrees about x axis
        extraRot << 1, 0, 0,
                    0, 0, -1,
                    0, 1, 0;

        // extraRot << 1, 0, 0,
        //             0, 1, 0,
        //             0, 0, 1;

        rotation = extraRot * rotation;
        //std::cout << "Rotation post" << rotation << std::endl;
        //fcl_position[2] -= armWidth * 1.5;

    }
    
    sejong::Quaternion finalJointOrientation(rotation);
    
    // Distance between joints
    double dist = calcDistance(joint1Pos, joint2Pos);
    std::cout << "Distance: " << dist << std::endl << std::endl;
    // Create a shape based on what type of appendage we have
    if(linkType == collisionLinkType::CLT_appendage_arm){
        collisionShape = std::make_shared<fcl::Cylinder<S> >(armWidth, dist);
    }
    else if(linkType == collisionLinkType::CLT_appendage_leg){
        collisionShape = std::make_shared<fcl::Cylinder<S> >(legWidth, dist);
    }

    // Populate transform
    // collisionTran = fcl::Transform3<S>::Identity();
    collisionTran.translation() = fcl_position;

    // TODO, this is really ugly but directly make shared doesn't work
    std::shared_ptr<fcl::CollisionGeometry<S>> temp(collisionShape.get()); 
    collisionGeo = temp;
    collisionGeo->setUserData(this);

    // TODO this is also really ugly
    if(collisionObj != NULL)
        delete collisionObj;
    collisionObj = new fcl::CollisionObject<S>(collisionGeo, collisionTran);
    collisionObj->setUserData(this);
    collisionObj->setQuatRotation(finalJointOrientation);


    // Create marker if desired
    #if DEBUG
    if(linkType == collisionLinkType::CLT_appendage_arm){
        jointMarker = createCylinder(armWidth, dist);
    }
    else if(linkType == collisionLinkType::CLT_appendage_leg){
        jointMarker = createCylinder(legWidth, dist);   
    }

    jointMarker.pose.position.x = fcl_position[0];
    jointMarker.pose.position.y = fcl_position[1];
    jointMarker.pose.position.z = fcl_position[2];

    jointMarker.pose.orientation.x = finalJointOrientation.x();
    jointMarker.pose.orientation.y = finalJointOrientation.y();
    jointMarker.pose.orientation.z = finalJointOrientation.z();
    jointMarker.pose.orientation.w = finalJointOrientation.w();
    #endif

    return collisionObj;
}

/**
 * Special function for the torso/head
 */
template<typename S>
fcl::CollisionObject<S>* CollisionLink<S>::computeCollisionObject(sejong::Vector& robot_q, double x_size, double y_size, double z_size) {
    // Get positions
    sejong::Vect3 joint1Pos;
    sejong::Vect3 joint2Pos;
    robot_model->getPosition(robot_q, link1, joint1Pos);
    robot_model->getPosition(robot_q, link2, joint2Pos);
    
    sejong::pretty_print(joint1Pos, std::cout, "Joint: " + std::to_string(link1) + " ");
    sejong::pretty_print(joint2Pos, std::cout, "Joint: " + std::to_string(link2) + " ");

    
    // Get orientations
    sejong::Quaternion joint1Orien;
    robot_model->getOrientation(robot_q, link1, joint1Orien);

    if(linkType == collisionLinkType::CLT_torso){
        // get hip points specially
        sejong::Vect3 temphip1;
        sejong::Vect3 temphip2;
        robot_model->getPosition(robot_q, SJLinkID::LK_leftHipYawLink, temphip1);
        robot_model->getPosition(robot_q, SJLinkID::LK_rightHipYawLink, temphip2);
        joint1Pos = calcMidpoint(temphip1, temphip2);
    }
    else if(linkType == collisionLinkType::CLT_head){

    }
    
    


    sejong::Vect3 fcl_position = calcMidpoint(joint1Pos, joint2Pos);

    Eigen::Matrix3d rotation = joint1Orien.normalized().toRotationMatrix();
    
    sejong::Quaternion finalJointOrientation(rotation);
    
    // Distance between joints
    double dist = calcDistance(joint1Pos, joint2Pos);
    std::cout << "Distance: " << dist << std::endl << std::endl;

    collisionShape = std::make_shared<fcl::Box<S> >(x_size, y_size, z_size);


    // Populate transform
    // collisionTran = fcl::Transform3<S>::Identity();
    collisionTran.translation() = fcl_position;

    // TODO, this is really ugly but directly make shared doesn't work
    std::shared_ptr<fcl::CollisionGeometry<S>> temp(collisionShape.get()); 
    collisionGeo = temp;
    collisionGeo->setUserData(this);

    // TODO this is also really ugly
    if(collisionObj != NULL)
        delete collisionObj;
    collisionObj = new fcl::CollisionObject<S>(collisionGeo, collisionTran);
    collisionObj->setUserData(this);
    collisionObj->setQuatRotation(finalJointOrientation);


    // Create marker if desired, need to fix this
    #if DEBUG
    jointMarker = createBox(x_size, y_size, z_size);
    // std::cout << "sizes" << std::endl;
    // std::cout << x_size << std::endl;
    // std::cout << y_size << std::endl;
    // std::cout << z_size << std::endl;

    jointMarker.pose.position.x = fcl_position[0];
    jointMarker.pose.position.y = fcl_position[1];
    jointMarker.pose.position.z = fcl_position[2];

    jointMarker.pose.orientation.x = finalJointOrientation.x();
    jointMarker.pose.orientation.y = finalJointOrientation.y();
    jointMarker.pose.orientation.z = finalJointOrientation.z();
    jointMarker.pose.orientation.w = finalJointOrientation.w();
    // std::cout << jointMarker << std::endl;
    // sejong::pretty_print(fcl_position, std::cout, "fcl_position");
    #endif

    return collisionObj;
}



#endif
