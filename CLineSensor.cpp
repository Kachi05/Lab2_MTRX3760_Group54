//-----------------------------------------------------------------------------
// CLineSensor.cpp
//
// Implements the binary floor-line sensor. The sensor's robot-relative offset
// is transformed into world coordinates, then its distance from each segment of
// the closed line is tested against the 2.5-unit half-width.
//-----------------------------------------------------------------------------

#include "CLineSensor.h"

#include <cmath>
#include <vector>

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
bool CLineSensor::Sense(
    const CPose& arRobotPose,
    const CLoopReader& arLoop ) const
{
    // Unit vector pointing in the robot's forward direction.
    const float ForwardX =
        std::cos( arRobotPose.mHeading );

    const float ForwardY =
        std::sin( arRobotPose.mHeading );

    // Unit vector pointing toward the robot's right-hand side.
    const float RightX =
        -std::sin( arRobotPose.mHeading );

    const float RightY =
        std::cos( arRobotPose.mHeading );

    // Convert the fixed sensor offset from robot coordinates to world position.
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

    // The loop reader stores corners; consecutive vertices form line segments.
    const std::vector<Vec2D>& Vertices =
        arLoop.GetVertices();

    if( !Vertices.empty() )
    {
        // Starting with the last vertex closes the final-to-first segment.
        Vec2D Previous =
            Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            const float Distance =
                DistanceToSegment(
                    SensorPosition,
                    Previous,
                    Vertex );

            // The full line is 5 units wide, so 2.5 units is its edge.
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
float CLineSensor::DistanceToSegment(
    Vec2D aPoint,
    Vec2D aSegmentStart,
    Vec2D aSegmentEnd ) const
{
    // Vector along the segment.
    const float VX =
        aSegmentEnd.x - aSegmentStart.x;

    const float VY =
        aSegmentEnd.y - aSegmentStart.y;

    // Vector from the segment start to the sensor point.
    const float WX =
        aPoint.x - aSegmentStart.x;

    const float WY =
        aPoint.y - aSegmentStart.y;

    const float LengthSquared =
        VX * VX + VY * VY;

    float T = 0.0f;

    // Project the point onto the infinite line when the segment is non-zero.
    if( LengthSquared > 0.0f )
    {
        T =
            ( WX * VX + WY * VY )
            / LengthSquared;
    }

    // Clamp the projection so the closest point stays on the finite segment.
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

    // Return the Euclidean distance to the closest point on the segment.
    const float DX =
        aPoint.x - ClosestPoint.x;

    const float DY =
        aPoint.y - ClosestPoint.y;

    return std::sqrt(
        DX * DX + DY * DY );
}
