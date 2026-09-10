//-----------------------------------------------------------------------------
// CLineSensor.cpp
//
// Implements the line sensor used by the line-following robot.
// The sensor position is transformed into world coordinates and compared
// with each segment of the floor line.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#include "CLineSensor.h"

#include <cmath>
#include <vector>

//-----------------------------------------------------------------------------
// Creates the sensor at a fixed position relative to the robot.
//-----------------------------------------------------------------------------
CLineSensor::CLineSensor(
    float aForwardOffset,
    float aRightOffset )
    :
        mForwardOffset( aForwardOffset ),
        mRightOffset( aRightOffset ),
        mLineHalfWidth( 2.5f )
{
}


//-----------------------------------------------------------------------------
// Checks whether the sensor is positioned over any segment of the floor line.
//-----------------------------------------------------------------------------
bool CLineSensor::Sense(
    const CPose& arRobotPose,
    const CLoopReader& arLoop ) const
{
    // Forward direction of the robot.
    const float ForwardX =
        std::cos( arRobotPose.mHeading );

    const float ForwardY =
        std::sin( arRobotPose.mHeading );

    // Right direction of the robot.
    const float RightX =
        -std::sin( arRobotPose.mHeading );

    const float RightY =
        std::cos( arRobotPose.mHeading );

    // Convert the sensor's position from robot-relative
    // coordinates into world coordinates.
    const Vec2D SensorPosition =
    {
        arRobotPose.mPosition.x
            + mForwardOffset * ForwardX
            + mRightOffset * RightX,

        arRobotPose.mPosition.y
            + mForwardOffset * ForwardY
            + mRightOffset * RightY
    };

    bool OnLine = false;

    const std::vector<Vec2D>& Vertices =
        arLoop.GetVertices();

    if( !Vertices.empty() )
    {
        // Start with the last vertex so the closing segment
        // between the last and first vertices is also checked.
        Vec2D Previous =
            Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            const float Distance =
                DistanceToSegment(
                    SensorPosition,
                    Previous,
                    Vertex );

            if( Distance <= mLineHalfWidth )
            {
                OnLine = true;
            }

            Previous = Vertex;
        }
    }

    return OnLine;
}


//-----------------------------------------------------------------------------
// Finds the shortest distance from a point to a finite line segment.
//-----------------------------------------------------------------------------
float CLineSensor::DistanceToSegment(
    Vec2D aPoint,
    Vec2D aSegmentStart,
    Vec2D aSegmentEnd ) const
{
    const float VX =
        aSegmentEnd.x - aSegmentStart.x;

    const float VY =
        aSegmentEnd.y - aSegmentStart.y;

    const float WX =
        aPoint.x - aSegmentStart.x;

    const float WY =
        aPoint.y - aSegmentStart.y;

    const float LengthSquared =
        VX * VX + VY * VY;

    // Find where the point projects along the line segment.
    float T = 0.0f;

    if( LengthSquared > 0.0f )
    {
        T =
            ( WX * VX + WY * VY )
            / LengthSquared;
    }

    // Keep the closest point between the two segment endpoints.
    if( T < 0.0f )
    {
        T = 0.0f;
    }
    else if( T > 1.0f )
    {
        T = 1.0f;
    }

    const Vec2D ClosestPoint =
    {
        aSegmentStart.x + T * VX,
        aSegmentStart.y + T * VY
    };

    const float DX =
        aPoint.x - ClosestPoint.x;

    const float DY =
        aPoint.y - ClosestPoint.y;

    return std::sqrt(
        DX * DX + DY * DY );
}