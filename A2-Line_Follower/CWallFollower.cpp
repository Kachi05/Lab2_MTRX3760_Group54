//-----------------------------------------------------------------------------
// CWallFollower.cpp
//
// Implements the wall-following controller.
// The right and diagonal range-sensor readings are used to calculate the
// wheel speeds required to follow the room walls.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#include "CWallFollower.h"

#include <cmath>

//-----------------------------------------------------------------------------
// Sets the sensor directions and wall-following control values.
//-----------------------------------------------------------------------------
CWallFollower::CWallFollower()
    :
        mRightSensor( float( M_PI ) / 2.0f ),
        mDiagonalSensor( float( M_PI ) / 4.0f ),

        mBaseSpeed( 35.0f ),

        mTargetDistance( 40.0f ),
        mDangerDistance( 55.0f ),
        mWallLostDistance( 80.0f ),

        mGain( 0.8f ),
        mMaximumCorrection( 25.0f ),

        mTurnSlowSpeed( -5.0f ),
        mTurnFastSpeed( 60.0f )
{
}


//-----------------------------------------------------------------------------
// Reads both range sensors and determines the wheel speeds for this update.
//-----------------------------------------------------------------------------
void CWallFollower::GetWheelSpeeds(
    const CPose& arRobotPose,
    const CLoopReader& arLoop,
    float& arLeftSpeed,
    float& arRightSpeed ) const
{
    const float RightDistance =
        mRightSensor.Sense(
            arRobotPose,
            arLoop );

    const float DiagonalDistance =
        mDiagonalSensor.Sense(
            arRobotPose,
            arLoop );

    // Inside corner approaching:
    // turn left away from wall ahead.
    if( DiagonalDistance < mDangerDistance )
    {
        arLeftSpeed = mTurnSlowSpeed;
        arRightSpeed = mTurnFastSpeed;
    }

    // Right wall disappeared:
    // turn right to find it again.
    else if( RightDistance > mWallLostDistance )
    {
        arLeftSpeed = mTurnFastSpeed;
        arRightSpeed = mTurnSlowSpeed;
    }

    // Normal wall following.
    else
    {
        // Calculate the error from the target wall distance.
        const float Error =
            RightDistance - mTargetDistance;

        float Correction =
            mGain * Error;

        // Limit the correction to avoid excessively large steering changes.
        if( Correction > mMaximumCorrection )
        {
            Correction = mMaximumCorrection;
        }
        else if( Correction < -mMaximumCorrection )
        {
            Correction = -mMaximumCorrection;
        }

        arLeftSpeed =
            mBaseSpeed + Correction;

        arRightSpeed =
            mBaseSpeed - Correction;
    }
}