//-----------------------------------------------------------------------------
// CLineFollower.h
//
// Declares the controller used by line-following robots. The controller owns
// two line sensors and converts their on/off readings into left and right wheel
// speeds so the robot follows the floor line.
//-----------------------------------------------------------------------------

#ifndef CLINEFOLLOWER_H
#define CLINEFOLLOWER_H

#include "CLineSensor.h"

class CLineFollower
{
    public:

        // Sets the sensor positions and the wheel speeds used by the controller.
        CLineFollower();

        // Reads both line sensors and chooses the wheel speeds for this update.
        void GetWheelSpeeds(
            const CPose& arRobotPose,
            const CLoopReader& arLoop,
            float& arLeftSpeed,
            float& arRightSpeed ) const;

    private:

        // Two sensors mounted near the front of the robot.
        CLineSensor mMainSensor;
        CLineSensor mSideSensor;

        // Speeds used for straight driving and corrective turns.
        const float mBaseSpeed;
        const float mSlowSpeed;
        const float mFastSpeed;
};

#endif
