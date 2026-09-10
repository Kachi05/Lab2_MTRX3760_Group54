//-----------------------------------------------------------------------------
// CRobot.cpp
//
// Implements robot movement, collision detection, trail storage, and drawing.
// The robot is modelled as a circular differential-drive robot with two
// independently controlled wheels.
//-----------------------------------------------------------------------------

#include "CRobot.h"

#include <cmath>
#include <iostream>
#include <vector>

//-----------------------------------------------------------------------------
CRobot::CRobot(
    CRender& arRender,
    const CPose& arStartPose )
    :
        // Begin at the pose read from SimpleWalls.map.
        mPose( arStartPose ),
        mCollisionCount( 0 ),
        mWasColliding( false ),

        // A1 specifies a robot radius of 15 units.
        mRadius( 15.0f ),
        mWheelSeparation( 30.0f ),
        mrRender( arRender )
{
    // Store the initial position as the first point of the trail.
    mTrail.push_back( mPose.mPosition );
}

//-----------------------------------------------------------------------------
void CRobot::Update(
    float aLeftSpeed,
    float aRightSpeed,
    float aTimeStep )
{
    // Average wheel speed gives the forward distance travelled this update.
    const float Distance =
        ( aLeftSpeed + aRightSpeed )
        * 0.5f
        * aTimeStep;

    // Difference in wheel speeds determines the robot's change in heading.
    const float HeadingChange =
        ( aLeftSpeed - aRightSpeed )
        / mWheelSeparation
        * aTimeStep;

    // Apply half of the turn before translating the robot.
    mPose.mHeading +=
        HeadingChange * 0.5f;

    // Move the robot in the direction it is currently facing.
    mPose.mPosition.x +=
        Distance
        * std::cos( mPose.mHeading );

    mPose.mPosition.y +=
        Distance
        * std::sin( mPose.mHeading );

    // Apply the remaining half of the heading change.
    mPose.mHeading +=
        HeadingChange * 0.5f;

    // Keep every new position so the trail remains visible for the full run.
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
        // Begin with the final vertex so the wall loop is closed.
        Vec2D Previous =
            Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            // The robot collides when its centre is less than one radius from a wall.
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

        // A1 requires each collision to be reported to the console.
        std::cout
            << "Collision "
            << mCollisionCount
            << std::endl;
    }

    // Remember the contact state for the next simulation update.
    mWasColliding = IsColliding;
}

//-----------------------------------------------------------------------------
float CRobot::DistanceToSegment(
    Vec2D aPoint,
    Vec2D aSegmentStart,
    Vec2D aSegmentEnd ) const
{
    // Direction vector of the wall segment.
    const float VX =
        aSegmentEnd.x - aSegmentStart.x;

    const float VY =
        aSegmentEnd.y - aSegmentStart.y;

    // Vector from the wall start to the point being tested.
    const float WX =
        aPoint.x - aSegmentStart.x;

    const float WY =
        aPoint.y - aSegmentStart.y;

    const float LengthSquared =
        VX * VX + VY * VY;

    float T = 0.0f;

    // Project the point onto the infinite line containing the wall segment.
    if( LengthSquared > 0.0f )
    {
        T =
            ( WX * VX + WY * VY )
            / LengthSquared;
    }

    // Clamp the projection so the closest point remains on the finite segment.
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

    // Return the Euclidean distance to the closest point on the segment.
    return std::sqrt(
        DX * DX + DY * DY );
}

//-----------------------------------------------------------------------------
void CRobot::Draw() const
{
    const float TrailThickness = 2.0f;
    const float HeadingThickness = 3.0f;

    //---Trail---
    // Join each stored position to the next to show the complete trajectory.
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
    // Draw the robot as the required radius-15 circle.
    mrRender.DrawCircle(
        mPose.mPosition,
        static_cast<int>( mRadius ),
        BLUE );

    //---Heading indicator---
    // Calculate a point one radius in front of the robot.
    const Vec2D HeadingEnd =
    {
        mPose.mPosition.x
            + mRadius
            * std::cos( mPose.mHeading ),

        mPose.mPosition.y
            + mRadius
            * std::sin( mPose.mHeading )
    };

    // Draw a line from the centre to show which way the robot is facing.
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
