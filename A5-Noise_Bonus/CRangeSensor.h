//-----------------------------------------------------------------------------
// CRangeSensor.h
//
// Declares a range sensor used by the wall-following controller. A sensor casts
// a ray at a fixed angle relative to the robot and reports the distance to the
// first room-wall segment that the ray intersects.
//-----------------------------------------------------------------------------

#ifndef CRANGESENSOR_H
#define CRANGESENSOR_H

#include "CLoopReader.h"

class CRangeSensor
{
    public:

        // Stores the sensor angle relative to the robot's current heading.
        CRangeSensor( float aAngleOffset );

        // Returns the nearest wall distance measured by this sensor.
        float Sense(
            const CPose& arRobotPose,
            const CLoopReader& arLoop ) const;

    private:

        // Tests one sensor ray against one finite wall segment.
        bool RaySegmentIntersection(
            Vec2D aOrigin,
            float aRayAngle,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd,
            float& arDistance ) const;

        const float mAngleOffset;
};

#endif
