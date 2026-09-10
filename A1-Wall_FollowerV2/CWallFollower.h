//-----------------------------------------------------------------------------
// CWallFollower.h
//
// Declares the wall-following controller. It owns the two required range
// sensors and converts their measurements into left and right wheel speeds.
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWER_H
#define CWALLFOLLOWER_H

#include "CRangeSensor.h"

//-----------------------------------------------------------------------------
// Controls the robot so that it follows the wall on its right-hand side.
//-----------------------------------------------------------------------------
class CWallFollower
{
    public:

        // Configures the two sensor directions and controller tuning values.
        CWallFollower();

        // Reads both sensors and selects wheel speeds for the current update.
        void GetWheelSpeeds(
            const CPose& arRobotPose,
            const CLoopReader& arLoop,
            float& arLeftSpeed,
            float& arRightSpeed ) const;

    private:

        // Required right-facing and forward-right range sensors.
        CRangeSensor mRightSensor;
        CRangeSensor mDiagonalSensor;

        // Normal forward speed and desired wall-following distances.
        const float mBaseSpeed;
        const float mTargetDistance;
        const float mDangerDistance;
        const float mWallLostDistance;

        // Proportional steering correction and its maximum magnitude.
        const float mGain;
        const float mMaximumCorrection;

        // Wheel speeds used for stronger cornering/search turns.
        const float mTurnSlowSpeed;
        const float mTurnFastSpeed;
};

#endif
