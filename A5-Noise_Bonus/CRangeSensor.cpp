//-----------------------------------------------------------------------------
// CRangeSensor.cpp
//
// Implements range sensing for the wall follower. Every segment of the closed
// wall loop is tested against the sensor ray and the closest valid intersection
// is returned as the measured range.
//-----------------------------------------------------------------------------

#include "CRangeSensor.h"

#include <cmath>
#include <vector>

//-----------------------------------------------------------------------------
CRangeSensor::CRangeSensor( float aAngleOffset )
    :
        mAngleOffset( aAngleOffset )
{
}

//-----------------------------------------------------------------------------
float CRangeSensor::Sense(
    const CPose& arRobotPose,
    const CLoopReader& arLoop ) const
{
    // Used when no wall is intersected by the sensor ray.
    const float MaximumRange = 10000.0f;

    float ClosestDistance = MaximumRange;

    const std::vector<Vec2D>& Vertices =
        arLoop.GetVertices();

    if( !Vertices.empty() )
    {
        // Begin at the last vertex so the final-to-first wall is also checked.
        Vec2D Previous = Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            float Distance = 0.0f;

            // Test this wall segment against the sensor's world-space ray.
            if( RaySegmentIntersection(
                    arRobotPose.mPosition,
                    arRobotPose.mHeading + mAngleOffset,
                    Previous,
                    Vertex,
                    Distance ) )
            {
                // The first wall hit is the valid intersection with least range.
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
    // Unit direction vector for the sensor ray.
    const float RayX = std::cos( aRayAngle );
    const float RayY = std::sin( aRayAngle );

    // Direction vector of the wall segment.
    const float SegmentX =
        aSegmentEnd.x - aSegmentStart.x;

    const float SegmentY =
        aSegmentEnd.y - aSegmentStart.y;

    // Cross-product denominator; near zero means parallel lines.
    const float Denominator =
        RayX * SegmentY - RayY * SegmentX;

    if( std::abs( Denominator ) < 0.000001f )
    {
        return false;
    }

    const float QX =
        aSegmentStart.x - aOrigin.x;

    const float QY =
        aSegmentStart.y - aOrigin.y;

    // T locates the intersection along the ray; U locates it on the segment.
    const float T =
        ( QX * SegmentY - QY * SegmentX )
        / Denominator;

    const float U =
        ( QX * RayY - QY * RayX )
        / Denominator;

    // T must be forward on the ray and U must lie on the finite wall segment.
    if( T >= 0.0f && U >= 0.0f && U <= 1.0f )
    {
        arDistance = T;
        return true;
    }

    return false;
}
