//-----------------------------------------------------------------------------
// CRobot.cpp
//
// Implements differential-drive robot motion, drawing and collision geometry.
// For A5, each robot begins with a small random pose offset and each wheel gets
// an independent random travel offset on every update, causing trajectories to
// spread while the original A2 controllers continue to react to their sensors.
//-----------------------------------------------------------------------------

#include "CRobot.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

//-----------------------------------------------------------------------------
CRobot::CRobot(
    CRender& arRender,
    const CPose& arStartPose,
    Color aTrailColour )
    :
        mPose( arStartPose ),
        mTrailColour( aTrailColour ),
        mCollisionCount( 0 ),
        mWasColliding( false ),
        mRadius( 15.0f ),
        mWheelSeparation( 30.0f ),

        // A5 noise settings. These can be tuned if the spread is too large/small.
        mStartPositionNoise( 3.0f ),
        mStartHeadingNoise( 0.05f ),
        mWheelNoise( 0.04f ),

        mrRender( arRender )
{
    // Give each robot a slightly different initial x/y position.
    mPose.mPosition.x +=
        RandomOffset( mStartPositionNoise );

    mPose.mPosition.y +=
        RandomOffset( mStartPositionNoise );

    // Give each robot a slightly different starting orientation.
    mPose.mHeading +=
        RandomOffset( mStartHeadingNoise );

    // The first trail point records the robot's perturbed A5 starting position.
    mTrail.push_back( mPose.mPosition );
}


//-----------------------------------------------------------------------------
float CRobot::RandomOffset(
    float aMaximumMagnitude ) const
{
    // Convert rand() to [0,1], then scale it symmetrically around zero.
    const float UnitRandom =
        float( std::rand() ) / RAND_MAX;

    return
        ( 2.0f * UnitRandom - 1.0f )
        * aMaximumMagnitude;
}


//-----------------------------------------------------------------------------
void CRobot::Update(
    float aLeftSpeed,
    float aRightSpeed,
    float aTimeStep )
{
    // Ideal distance each wheel would travel during this fixed simulation step.
    float LeftDistance =
        aLeftSpeed * aTimeStep;

    float RightDistance =
        aRightSpeed * aTimeStep;

    // A5 requirement: independently perturb how far each wheel actually turns.
    LeftDistance +=
        RandomOffset( mWheelNoise );

    RightDistance +=
        RandomOffset( mWheelNoise );

    // Centre translation is the mean distance travelled by both wheels.
    const float Distance =
        ( LeftDistance + RightDistance )
        * 0.5f;

    // Unequal wheel travel rotates the differential-drive robot.
    const float HeadingChange =
        ( LeftDistance - RightDistance )
        / mWheelSeparation;

    // Apply half the rotation before translation for a smoother motion estimate.
    mPose.mHeading +=
        HeadingChange * 0.5f;

    mPose.mPosition.x +=
        Distance
        * std::cos( mPose.mHeading );

    mPose.mPosition.y +=
        Distance
        * std::sin( mPose.mHeading );

    // Apply the remaining half of the rotation after translation.
    mPose.mHeading +=
        HeadingChange * 0.5f;

    // Keep the complete noisy trajectory visible for the whole simulation.
    mTrail.push_back(
        mPose.mPosition );
}


//-----------------------------------------------------------------------------
void CRobot::CheckCollision(
    const CLoopReader& arLoop )
{
    // This function is retained from A2 for reuse, although A5 ignores collisions.
    bool IsColliding = false;

    const std::vector<Vec2D>& Vertices =
        arLoop.GetVertices();

    if( !Vertices.empty() )
    {
        Vec2D Previous =
            Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            // A collision occurs when the robot centre is within its radius of a wall.
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

    // Count a new collision only when the robot first enters contact with a wall.
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
    // Direction of the wall segment.
    const float VX =
        aSegmentEnd.x - aSegmentStart.x;

    const float VY =
        aSegmentEnd.y - aSegmentStart.y;

    // Direction from the segment start to the test point.
    const float WX =
        aPoint.x - aSegmentStart.x;

    const float WY =
        aPoint.y - aSegmentStart.y;

    const float LengthSquared =
        VX * VX + VY * VY;

    float T = 0.0f;

    // Project the point onto the segment's infinite line when possible.
    if( LengthSquared > 0.0f )
    {
        T =
            ( WX * VX + WY * VY )
            / LengthSquared;
    }

    // Clamp to the finite segment endpoints.
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

    // Draw every stored pair of points so each robot's entire path remains visible.
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
                mTrailColour );

            Previous = mTrail[i];
        }
    }

    // Draw the robot as the required radius-15 disc.
    mrRender.DrawCircle(
        mPose.mPosition,
        static_cast<int>( mRadius ),
        BLUE );

    // Calculate the end of the line that shows the robot's current heading.
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
