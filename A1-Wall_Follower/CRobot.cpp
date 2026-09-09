#include "CRobot.h"

#include <cmath>
#include <iostream>
#include <vector>

//-----------------------------------------------------------------------------
CRobot::CRobot(
    CRender& arRender,
    const CPose& arStartPose )
    :
        mPose( arStartPose ),
        mCollisionCount( 0 ),
        mWasColliding( false ),
        mRadius( 15.0f ),
        mWheelSeparation( 30.0f ),
        mrRender( arRender )
{
    mTrail.push_back( mPose.mPosition );
}

//-----------------------------------------------------------------------------
void CRobot::Update(
    float aLeftSpeed,
    float aRightSpeed,
    float aTimeStep )
{
    const float Distance =
        ( aLeftSpeed + aRightSpeed )
        * 0.5f
        * aTimeStep;

    const float HeadingChange =
        ( aLeftSpeed - aRightSpeed )
        / mWheelSeparation
        * aTimeStep;

    // Half the turn.
    mPose.mHeading +=
        HeadingChange * 0.5f;

    // Drive.
    mPose.mPosition.x +=
        Distance
        * std::cos( mPose.mHeading );

    mPose.mPosition.y +=
        Distance
        * std::sin( mPose.mHeading );

    // Remaining half of turn.
    mPose.mHeading +=
        HeadingChange * 0.5f;

    mTrail.push_back(
        mPose.mPosition );
}

//-----------------------------------------------------------------------------
void CRobot::CheckCollision(
    const CLoopReader& arLoop )
{
    bool IsColliding = false;

    const std::vector<Vec2D>& Vertices =
        arLoop.GetVertices();

    if( !Vertices.empty() )
    {
        Vec2D Previous =
            Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            if( DistanceToSegment(
                    mPose.mPosition,
                    Previous,
                    Vertex ) < mRadius )
            {
                IsColliding = true;
            }

            Previous = Vertex;
        }
    }

    // Count one collision event rather than
    // counting every frame while touching.
    if( IsColliding && !mWasColliding )
    {
        ++mCollisionCount;

        std::cout
            << "Collision "
            << mCollisionCount
            << std::endl;
    }

    mWasColliding = IsColliding;
}

//-----------------------------------------------------------------------------
float CRobot::DistanceToSegment(
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

    float T = 0.0f;

    if( LengthSquared > 0.0f )
    {
        T =
            ( WX * VX + WY * VY )
            / LengthSquared;
    }

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

//-----------------------------------------------------------------------------
void CRobot::Draw() const
{
    const float TrailThickness = 2.0f;
    const float HeadingThickness = 3.0f;

    //---Trail---
    if( mTrail.size() > 1 )
    {
        Vec2D Previous = mTrail[0];

        for( std::size_t i=1;
             i<mTrail.size();
             ++i )
        {
            mrRender.DrawLine(
                Previous,
                mTrail[i],
                TrailThickness,
                GREEN );

            Previous = mTrail[i];
        }
    }

    //---Robot body---
    mrRender.DrawCircle(
        mPose.mPosition,
        static_cast<int>( mRadius ),
        BLUE );

    //---Heading indicator---
    const Vec2D HeadingEnd =
    {
        mPose.mPosition.x
            + mRadius
            * std::cos( mPose.mHeading ),

        mPose.mPosition.y
            + mRadius
            * std::sin( mPose.mHeading )
    };

    mrRender.DrawLine(
        mPose.mPosition,
        HeadingEnd,
        HeadingThickness,
        RAYWHITE );
}

//-----------------------------------------------------------------------------
const CPose& CRobot::GetPose() const
{
    return mPose;
}

//-----------------------------------------------------------------------------
int CRobot::GetCollisionCount() const
{
    return mCollisionCount;
}
