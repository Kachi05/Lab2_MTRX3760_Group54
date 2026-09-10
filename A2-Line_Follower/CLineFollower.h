#ifndef CLINEFOLLOWER_H
#define CLINEFOLLOWER_H

#include "CLineSensor.h"

class CLineFollower
{
    public:

        CLineFollower();

        void GetWheelSpeeds(
            const CPose& arRobotPose,
            const CLoopReader& arLoop,
            float& arLeftSpeed,
            float& arRightSpeed ) const;

    private:

        CLineSensor mMainSensor;
        CLineSensor mSideSensor;

        const float mBaseSpeed;
        const float mSlowSpeed;
        const float mFastSpeed;
};

#endif