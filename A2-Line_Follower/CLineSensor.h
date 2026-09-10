#ifndef CLINESENSOR_H
#define CLINESENSOR_H

#include "CLoopReader.h"

class CLineSensor
{
    public:

        CLineSensor(
            float aForwardOffset,
            float aRightOffset );

        bool Sense(
            const CPose& arRobotPose,
            const CLoopReader& arLoop ) const;

    private:

        float DistanceToSegment(
            Vec2D aPoint,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd ) const;

        const float mForwardOffset;
        const float mRightOffset;

        const float mLineHalfWidth;
};

#endif