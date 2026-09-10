//-----------------------------------------------------------------------------
// CRangeSensor.h
//
// Declares the range sensor used by the wall-following robot. Each sensor casts
// a ray from the robot at a fixed angle and reports the distance to the first
// wall segment that the ray intersects.
//-----------------------------------------------------------------------------

#ifndef CRANGESENSOR_H
#define CRANGESENSOR_H

#include "CLoopReader.h"

//-----------------------------------------------------------------------------
// Represents one range sensor mounted at a fixed angle relative to the robot.
//-----------------------------------------------------------------------------
class CRangeSensor
{
    public:

        // Sets the direction of the sensor relative to the robot heading.
        CRangeSensor( float aAngleOffset );

        // Returns the distance from the robot to the nearest wall hit by
        // the sensor ray.
        float Sense(
            const CPose& arRobotPose,
            const CLoopReader& arLoop ) const;

    private:

        // Tests whether a sensor ray intersects one wall segment and, if so,
        // returns the distance from the ray origin to that intersection.
        bool RaySegmentIntersection(
            Vec2D aOrigin,
            float aRayAngle,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd,
            float& arDistance ) const;

        // Fixed sensor direction relative to the robot heading, in radians.
        const float mAngleOffset;
};

#endif
