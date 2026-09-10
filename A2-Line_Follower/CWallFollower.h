#ifndef CWALLFOLLOWER_H
#define CWALLFOLLOWER_H

#include "CRangeSensor.h"

class CWallFollower
{
    public:

        CWallFollower();

        void GetWheelSpeeds(
            const CPose& arRobotPose,
            const CLoopReader& arLoop,
            float& arLeftSpeed,
            float& arRightSpeed ) const;

    private:

        CRangeSensor mRightSensor;
        CRangeSensor mDiagonalSensor;

        const float mBaseSpeed;
        const float mTargetDistance;
        const float mDangerDistance;
        const float mWallLostDistance;

        const float mGain;
        const float mMaximumCorrection;

        const float mTurnSlowSpeed;
        const float mTurnFastSpeed;
};

#endif
