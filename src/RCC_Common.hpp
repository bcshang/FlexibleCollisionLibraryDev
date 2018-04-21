#ifndef RCC_COMMON_HPP
#define RCC_COMMON_HPP

#define NODE_RATE 10 // ROS loop rate in HZ

#define sizeBox .1 // size of collision box in m

#define DEBUG 1 // show debugging information?


enum collisionLinkType{
    CLT_appendage_arm = 0,
    CLT_appendage_leg,
    CLT_torso,
    CLT_head
};

// radius in m
#define armWidth .085
#define legWidth .11
#define torsoDepth .3

#endif

