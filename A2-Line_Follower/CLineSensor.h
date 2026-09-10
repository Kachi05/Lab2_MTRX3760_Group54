//-----------------------------------------------------------------------------
// CLineSensor.h
//
// Declares the CLineSensor class used by the line-following robot.
// The sensor is positioned relative to the robot and reports whether it is
// currently positioned over the floor line.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#ifndef CLINESENSOR_H
#define CLINESENSOR_H

#include "CLoopReader.h"
// Creates a line sensor at the given forward and right offsets.
class CLineSensor
{
    public:
        // Creates a line sensor at the given forward and right offsets.
        CLineSensor(
            float aForwardOffset,
            float aRightOffset );

        bool Sense(
            const CPose& arRobotPose,
            const CLoopReader& arLoop ) const;
    // Return the shortest distance from a point to a finite line segment.
    // Used to test the sensor against every edge of the closed floor-line loop.
    private:
        // Finds the shortest distance from a point to a line segment.
        float DistanceToSegment(
            Vec2D aPoint,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd ) const;
        // Sensor location in robot-local coordinates.
        const float mForwardOffset;
        const float mRightOffset;
        // The map stores the centreline, so half the specified line width
        // is used as the sensing threshold.
        const float mLineHalfWidth;
};

#endif