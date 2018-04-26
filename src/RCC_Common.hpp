#ifndef RCC_COMMON_HPP
#define RCC_COMMON_HPP

#define NODE_RATE 5 // ROS loop rate in HZ

#define sizeBox .1 // size of collision box in m

#define DEBUG 1 // show debugging information
#define LISTJOINTS 0
#define SHOWMARKERS 1

// CollisionLink typedefs
enum collisionLinkType{
    CLT_appendage_right_arm = 0,
    CLT_appendage_left_arm,
    CLT_appendage_leg,
    CLT_torso,
    CLT_head
};

// radius in meters
#define armWidth .085
#define legWidth .11

// How swoll is valkyrie?
#define torsoDepth .3

#endif

