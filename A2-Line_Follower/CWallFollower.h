//-----------------------------------------------------------------------------
// CWallFollower.h
//
// Declares the CWallFollower controller used by the wall-following robot.
// Two range sensors are used to maintain a suitable distance from the wall
// while allowing the robot to negotiate corners.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWER_H
#define CWALLFOLLOWER_H

#include "CRangeSensor.h"

//-----------------------------------------------------------------------------
// CWallFollower uses two range sensors to control the robot while keeping
// the wall on its right-hand side.
//-----------------------------------------------------------------------------
class CWallFollower
{
    public:

        // Creates the wall follower with its sensor and steering settings.
        CWallFollower();

        // Reads the wall distances and selects the required wheel speeds.
        void GetWheelSpeeds(
            const CPose& arRobotPose,
            const CLoopReader& arLoop,
            float& arLeftSpeed,
            float& arRightSpeed ) const;

    private:

        //---Range sensors---
        CRangeSensor mRightSensor;
        CRangeSensor mDiagonalSensor;

        //---Wall following settings---
        const float mBaseSpeed;
        const float mTargetDistance;
        const float mDangerDistance;
        const float mWallLostDistance;

        //---Steering correction---
        const float mGain;
        const float mMaximumCorrection;

        //---Cornering speeds---
        const float mTurnSlowSpeed;
        const float mTurnFastSpeed;
};

#endif