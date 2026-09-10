//-----------------------------------------------------------------------------
// CLineFollower.h
//
// Declares the CLineFollower controller used by the line-following robot.
// The controller uses two line sensors to determine the required left and
// right wheel speeds.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#ifndef CLINEFOLLOWER_H
#define CLINEFOLLOWER_H

#include "CLineSensor.h"

//-----------------------------------------------------------------------------
// CLineFollower uses two line sensors to control the robot's wheel speeds
// and keep the robot following the floor line.
//-----------------------------------------------------------------------------
class CLineFollower
{
    public:

        // Creates the line follower with its sensor and speed settings.
        CLineFollower();

        // Reads both sensors and selects the required wheel speeds.
        void GetWheelSpeeds(
            const CPose& arRobotPose,
            const CLoopReader& arLoop,
            float& arLeftSpeed,
            float& arRightSpeed ) const;

    private:

        //---Line sensors---
        CLineSensor mMainSensor;
        CLineSensor mSideSensor;

        //---Wheel speeds---
        const float mBaseSpeed;
        const float mSlowSpeed;
        const float mFastSpeed;
};

#endif