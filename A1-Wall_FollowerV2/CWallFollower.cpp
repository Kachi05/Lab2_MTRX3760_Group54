//-----------------------------------------------------------------------------
// CWallFollower.cpp
//
// Implements the wall-following control logic. The right and diagonal sensor
// measurements are used to steer away from nearby walls, search for a lost
// right-hand wall, or maintain the target distance during normal following.
//-----------------------------------------------------------------------------

#include "CWallFollower.h"

#include <cmath>

//-----------------------------------------------------------------------------
CWallFollower::CWallFollower()
    :
        // A1 requires one sensor directly right and one forward-right.
        mRightSensor( float( M_PI ) / 2.0f ),
        mDiagonalSensor( float( M_PI ) / 4.0f ),

        // Controller tuning values used for the supplied room.
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
void CWallFollower::GetWheelSpeeds(
    const CPose& arRobotPose,
    const CLoopReader& arLoop,
    float& arLeftSpeed,
    float& arRightSpeed ) const
{
    // Measure the wall directly to the robot's right.
    const float RightDistance =
        mRightSensor.Sense(
            arRobotPose,
            arLoop );

    // Measure forward-right to detect an approaching corner or wall.
    const float DiagonalDistance =
        mDiagonalSensor.Sense(
            arRobotPose,
            arLoop );

    // Inside corner approaching:
    // turn left away from the nearby wall ahead.
    if( DiagonalDistance < mDangerDistance )
    {
        arLeftSpeed = mTurnSlowSpeed;
        arRightSpeed = mTurnFastSpeed;
    }

    // Right wall disappeared:
    // turn right until the robot finds the wall again.
    else if( RightDistance > mWallLostDistance )
    {
        arLeftSpeed = mTurnFastSpeed;
        arRightSpeed = mTurnSlowSpeed;
    }

    // Normal wall following.
    else
    {
        // Positive error means the robot is too far from the right-hand wall.
        const float Error =
            RightDistance - mTargetDistance;

        // Use proportional feedback to correct the robot's direction.
        float Correction =
            mGain * Error;

        // Limit the steering correction so turns are not excessively sharp.
        if( Correction > mMaximumCorrection )
        {
            Correction = mMaximumCorrection;
        }
        else if( Correction < -mMaximumCorrection )
        {
            Correction = -mMaximumCorrection;
        }

        // Different wheel speeds steer the robot while maintaining forward motion.
        arLeftSpeed =
            mBaseSpeed + Correction;

        arRightSpeed =
            mBaseSpeed - Correction;
    }
}
