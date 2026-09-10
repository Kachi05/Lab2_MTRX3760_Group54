//-----------------------------------------------------------------------------
// CLineSensor.h
//
// Declares a binary floor-line sensor. Each sensor has a fixed position relative
// to the robot and reports whether the floor directly underneath it lies within
// the 5-unit-wide line from SimpleLine.map.
//-----------------------------------------------------------------------------

#ifndef CLINESENSOR_H
#define CLINESENSOR_H

#include "CLoopReader.h"

class CLineSensor
{
    public:

        // Sensor position is specified relative to the robot body.
        CLineSensor(
            float aForwardOffset,
            float aRightOffset );

        // Returns true when the sensor is positioned over the floor line.
        bool Sense(
            const CPose& arRobotPose,
            const CLoopReader& arLoop ) const;

    private:

        // Finds the shortest distance from a point to one line segment.
        float DistanceToSegment(
            Vec2D aPoint,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd ) const;

        // Sensor location relative to the robot centre.
        const float mForwardOffset;
        const float mRightOffset;

        // Half of the required 5-unit line width.
        const float mLineHalfWidth;
};

#endif
