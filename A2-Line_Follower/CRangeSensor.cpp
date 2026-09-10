//-----------------------------------------------------------------------------
// CRangeSensor.cpp
//
// Implements the range sensor used by the wall-following robot.
// A ray is projected from the robot and tested against each wall segment to
// determine the closest detected wall distance.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#include "CRangeSensor.h"

#include <cmath>
#include <vector>

//-----------------------------------------------------------------------------
// Stores the sensor's angle relative to the robot.
//-----------------------------------------------------------------------------
CRangeSensor::CRangeSensor( float aAngleOffset )
    :
        mAngleOffset( aAngleOffset )
{
}


//-----------------------------------------------------------------------------
// Checks every wall segment and returns the closest detected distance.
//-----------------------------------------------------------------------------
float CRangeSensor::Sense(
    const CPose& arRobotPose,
    const CLoopReader& arLoop ) const
{
    // Used when no wall is detected by the sensor.
    const float MaximumRange = 10000.0f;

    float ClosestDistance = MaximumRange;

    const std::vector<Vec2D>& Vertices =
        arLoop.GetVertices();

    if( !Vertices.empty() )
    {
        Vec2D Previous = Vertices.back();

        // Test the sensor ray against every segment of the room.
        for( const Vec2D& Vertex : Vertices )
        {
            float Distance = 0.0f;

            if( RaySegmentIntersection(
                    arRobotPose.mPosition,
                    arRobotPose.mHeading + mAngleOffset,
                    Previous,
                    Vertex,
                    Distance ) )
            {
                if( Distance < ClosestDistance )
                {
                    ClosestDistance = Distance;
                }
            }

            Previous = Vertex;
        }
    }

    return ClosestDistance;
}


//-----------------------------------------------------------------------------
// Determines whether the sensor ray intersects a finite wall segment.
//-----------------------------------------------------------------------------
bool CRangeSensor::RaySegmentIntersection(
    Vec2D aOrigin,
    float aRayAngle,
    Vec2D aSegmentStart,
    Vec2D aSegmentEnd,
    float& arDistance ) const
{
    const float RayX = std::cos( aRayAngle );
    const float RayY = std::sin( aRayAngle );

    const float SegmentX =
        aSegmentEnd.x - aSegmentStart.x;

    const float SegmentY =
        aSegmentEnd.y - aSegmentStart.y;

    const float Denominator =
        RayX * SegmentY - RayY * SegmentX;

    // Parallel lines do not have a valid intersection.
    if( std::abs( Denominator ) < 0.000001f )
    {
        return false;
    }

    const float QX =
        aSegmentStart.x - aOrigin.x;

    const float QY =
        aSegmentStart.y - aOrigin.y;

    const float T =
        ( QX * SegmentY - QY * SegmentX )
        / Denominator;

    const float U =
        ( QX * RayY - QY * RayX )
        / Denominator;

    // The intersection must be in front of the sensor
    // and between the two endpoints of the wall segment.
    if( T >= 0.0f && U >= 0.0f && U <= 1.0f )
    {
        arDistance = T;
        return true;
    }

    return false;
}