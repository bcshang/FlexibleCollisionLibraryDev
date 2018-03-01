#ifndef VALKYRIE_DEFINITION
#define VALKYRIE_DEFINITION

#include <stdio.h>
#include <iostream>
#include <vector>
#define CONFIG_PATH THIS_COM""

#define MEASURE_TIME 0

#define _DEF_SENSOR_DATA_ double time, const std::vector<double> & jpos, const std::vector<double> & jvel, const std::vector<double> & torque, const sejong::Vect3 & body_pos, const sejong::Quaternion & body_ori, const sejong::Vect3 & body_vel, const sejong::Vect3 & ang_vel

#define _VAR_SENSOR_DATA_ time, jpos, jvel, torque, body_pos, body_ori, body_vel, ang_vel


enum SJLinkID{
    LK_pelvis = 0,
    LK_leftHipYawLink,
    LK_leftHipRollLink,
    LK_leftHipPitchLink,
    LK_leftKneePitchLink,
    LK_leftAnklePitchLink,
    LK_leftFoot,
    LK_rightHipYawLink,
    LK_rightHipRollLink,
    LK_rightHipPitchLink,
    LK_rightKneePitchLink,
    LK_rightAnklePitchLink,
    LK_rightFoot,
    LK_torsoYawLink,
    LK_torsoPitchLink,
    LK_torso = 15,
    LK_leftShoulderPitchLink,
    LK_leftShoulderRollLink,
    LK_leftShoulderYawLink,
    LK_leftElbowPitchLink,
    LK_leftForearmLink,
    /* LK_leftWristRollLink, */
    /* LK_leftPalm, */
    LK_lowerNeckPitchLink,
    LK_neckYawLink,
    LK_upperNeckPitchLink,
    LK_rightShoulderPitchLink,
    LK_rightShoulderRollLink,
    LK_rightShoulderYawLink,
    LK_rightElbowPitchLink,
    LK_rightForearmLink,
    /* LK_rightWristRollLink, */
    /* LK_rightPalm, */
    NUM_LINK,
    LK_leftCOP_Frame,
    LK_rightCOP_Frame,
    LK_leftFootOutFront,
    LK_leftFootOutBack,
    LK_leftFootInBack,
    LK_leftFootInFront,
    LK_rightFootOutFront,
    LK_rightFootOutBack,
    LK_rightFootInBack,
    LK_rightFootInFront
};

enum SJJointID{
    leftHipYaw = 0,
    leftHipRoll = 1  ,
    leftHipPitch = 2 ,
    leftKneePitch,
    leftAnklePitch,
    leftAnkleRoll,
    rightHipYaw   ,
    rightHipRoll  ,
    rightHipPitch ,
    rightKneePitch,
    rightAnklePitch,
    rightAnkleRoll       ,
    torsoYaw             ,
    torsoPitch           ,
    torsoRoll            ,
    leftShoulderPitch,
    leftShoulderRoll     ,
    leftShoulderYaw      ,
    leftElbowPitch       ,
    leftForearmYaw  ,
    /* leftWristRoll        , */
    /* leftWristPitch       , */
    lowerNeckPitch  , 
    neckYaw         , 
    upperNeckPitch  , 
    rightShoulderPitch, 
    rightShoulderRoll , 
    rightShoulderYaw  , 
    rightElbowPitch   , 
    rightForearmYaw 
    /* rightWristRoll    ,  */
    /* rightWristPitch    */
};

#define NUM_QDOT 34
#define NUM_VIRTUAL 6
#define NUM_Q (NUM_QDOT + 1)
#define NUM_ACT_JOINT (NUM_QDOT - NUM_VIRTUAL)

#endif
