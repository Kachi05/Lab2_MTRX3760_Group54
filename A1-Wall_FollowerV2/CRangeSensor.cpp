//-----------------------------------------------------------------------------
// CRangeSensor.cpp
//
// Implements the wall range sensor. The sensor checks its ray against every
// wall segment in the closed loop and returns the closest valid intersection.
//-----------------------------------------------------------------------------

#include "CRangeSensor.h"

#include <cmath>
#include <vector>

//-----------------------------------------------------------------------------
CRangeSensor::CRangeSensor( float aAngleOffset )
    :
        // Store the fixed direction of this sensor relative to the robot.
        mAngleOffset( aAngleOffset )
{
}

//-----------------------------------------------------------------------------
float CRangeSensor::Sense(
    const CPose& arRobotPose,
    const CLoopReader& arLoop ) const
{
    // Value returned if no wall is intersected by the sensor ray.
    const float MaximumRange = 10000.0f;

    float ClosestDistance = MaximumRange;

    // The vertices define the closed sequence of wall segments.
    const std::vector<Vec2D>& Vertices =
        arLoop.GetVertices();

    if( !Vertices.empty() )
    {
        // Start from the final vertex so the first test closes the loop.
        Vec2D Previous = Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            float Distance = 0.0f;

            // Test the sensor ray against the current wall segment.
            if( RaySegmentIntersection(
                    arRobotPose.mPosition,
                    arRobotPose.mHeading + mAngleOffset,
                    Previous,
                    Vertex,
                    Distance ) )
            {
                // Keep only the nearest wall hit by the ray.
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
bool CRangeSensor::RaySegmentIntersection(
    Vec2D aOrigin,
    float aRayAngle,
    Vec2D aSegmentStart,
    Vec2D aSegmentEnd,
    float& arDistance ) const
{
    // Unit direction vector of the sensor ray.
    const float RayX = std::cos( aRayAngle );
    const float RayY = std::sin( aRayAngle );

    // Direction vector of the wall segment.
    const float SegmentX =
        aSegmentEnd.x - aSegmentStart.x;

    const float SegmentY =
        aSegmentEnd.y - aSegmentStart.y;

    // Cross-product denominator used by the ray/segment intersection test.
    const float Denominator =
        RayX * SegmentY - RayY * SegmentX;

    // A near-zero value means the ray and segment are parallel.
    if( std::abs( Denominator ) < 0.000001f )
    {
        return false;
    }

    // Vector from the ray origin to the start of the wall segment.
    const float QX =
        aSegmentStart.x - aOrigin.x;

    const float QY =
        aSegmentStart.y - aOrigin.y;

    // T locates the intersection along the ray.
    const float T =
        ( QX * SegmentY - QY * SegmentX )
        / Denominator;

    // U locates the intersection along the finite wall segment.
    const float U =
        ( QX * RayY - QY * RayX )
        / Denominator;

    // A valid hit must lie in front of the ray and on the wall segment.
    if( T >= 0.0f && U >= 0.0f && U <= 1.0f )
    {
        arDistance = T;
        return true;
    }

    return false;
}
