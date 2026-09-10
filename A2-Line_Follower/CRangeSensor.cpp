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
    const float MaximumRange = 10000.0f;

    float ClosestDistance = MaximumRange;

    const std::vector<Vec2D>& Vertices =
        arLoop.GetVertices();

    if( !Vertices.empty() )
    {
        Vec2D Previous = Vertices.back();

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

    if( T >= 0.0f && U >= 0.0f && U <= 1.0f )
    {
        arDistance = T;
        return true;
    }

    return false;
}
