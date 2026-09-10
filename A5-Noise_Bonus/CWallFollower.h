//-----------------------------------------------------------------------------
// CWallFollower.h
//
// Declares the controller for wall-following robots. It owns the required two
// range sensors and uses their measurements to select left and right wheel
// speeds that keep the wall on the robot's right-hand side.
//-----------------------------------------------------------------------------

#ifndef CWALLFOLLOWER_H
#define CWALLFOLLOWER_H

#include "CRangeSensor.h"

class CWallFollower
{
    public:

        // Configures both sensor angles and all controller tuning values.
        CWallFollower();

        // Uses the current sensor readings to choose wheel speeds for one update.
        void GetWheelSpeeds(
            const CPose& arRobotPose,
            const CLoopReader& arLoop,
            float& arLeftSpeed,
            float& arRightSpeed ) const;

    private:

        // Required 90-degree right and 45-degree forward-right range sensors.
        CRangeSensor mRightSensor;
        CRangeSensor mDiagonalSensor;

        // Normal wall-following speed and distance thresholds.
        const float mBaseSpeed;
        const float mTargetDistance;
        const float mDangerDistance;
        const float mWallLostDistance;

        // Proportional correction settings used during normal wall following.
        const float mGain;
        const float mMaximumCorrection;

        // Speeds used to make stronger turns at corners or when the wall is lost.
        const float mTurnSlowSpeed;
        const float mTurnFastSpeed;
};

#endif
