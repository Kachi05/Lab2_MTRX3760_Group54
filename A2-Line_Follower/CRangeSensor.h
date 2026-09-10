#ifndef CRANGESENSOR_H
#define CRANGESENSOR_H

#include "CLoopReader.h"

class CRangeSensor
{
    public:

        CRangeSensor( float aAngleOffset );

        float Sense(
            const CPose& arRobotPose,
            const CLoopReader& arLoop ) const;

    private:

        bool RaySegmentIntersection(
            Vec2D aOrigin,
            float aRayAngle,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd,
            float& arDistance ) const;

        const float mAngleOffset;
};

#endif
