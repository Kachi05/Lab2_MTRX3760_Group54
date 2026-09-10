//-----------------------------------------------------------------------------
// CRangeSensor.h
//
// Declares the CRangeSensor class used by the wall-following robot.
// The sensor measures the distance from the robot to the nearest wall in a
// specified direction.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#ifndef CRANGESENSOR_H
#define CRANGESENSOR_H

#include "CLoopReader.h"

//-----------------------------------------------------------------------------
// CRangeSensor models a distance sensor aimed at a fixed angle relative
// to the robot. It reports the distance to the nearest detected wall.
//-----------------------------------------------------------------------------
class CRangeSensor
{
    public:

        // Creates a range sensor at the given angle relative to the robot.
        CRangeSensor( float aAngleOffset );

        // Returns the distance to the nearest wall detected by the sensor.
        float Sense(
            const CPose& arRobotPose,
            const CLoopReader& arLoop ) const;

    private:

        // Checks whether the sensor ray intersects a wall segment.
        bool RaySegmentIntersection(
            Vec2D aOrigin,
            float aRayAngle,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd,
            float& arDistance ) const;

        //---Sensor angle---
        const float mAngleOffset;
};

#endif