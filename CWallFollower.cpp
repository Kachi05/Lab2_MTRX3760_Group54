//-----------------------------------------------------------------------------
// CWallFollower.cpp
//
// Implements the wall-following controller. The right sensor maintains wall
// distance while the diagonal sensor detects approaching corners. A5 noise is
// applied by CRobot after these intended wheel speeds are calculated.
//-----------------------------------------------------------------------------

#include "CWallFollower.h"

#include <cmath>

//-----------------------------------------------------------------------------
CWallFollower::CWallFollower()
    :
        // Positive angles rotate clockwise in the supplied screen coordinates.
        mRightSensor( float( M_PI ) / 2.0f ),
        mDiagonalSensor( float( M_PI ) / 4.0f ),

        mBaseSpeed( 35.0f ),

        // Desired wall range and thresholds used to react to the environment.
        mTargetDistance( 40.0f ),
        mDangerDistance( 55.0f ),
        mWallLostDistance( 80.0f ),

        // Limits for proportional steering during ordinary wall following.
        mGain( 0.8f ),
        mMaximumCorrection( 25.0f ),

        // Strong-turn speeds used at corners and when reacquiring the wall.
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
    // Measure the wall directly right of the robot and forward-right of it.
    const float RightDistance =
        mRightSensor.Sense(
            arRobotPose,
            arLoop );

    const float DiagonalDistance =
        mDiagonalSensor.Sense(
            arRobotPose,
            arLoop );

    // A nearby diagonal wall indicates an approaching inside corner: turn left.
    if( DiagonalDistance < mDangerDistance )
    {
        arLeftSpeed = mTurnSlowSpeed;
        arRightSpeed = mTurnFastSpeed;
    }

    // A very large right range means the wall has disappeared: turn right.
    else if( RightDistance > mWallLostDistance )
    {
        arLeftSpeed = mTurnFastSpeed;
        arRightSpeed = mTurnSlowSpeed;
    }

    // Otherwise use proportional steering to maintain the target wall distance.
    else
    {
        const float Error =
            RightDistance - mTargetDistance;

        float Correction =
            mGain * Error;

        // Limit the correction so one unusual reading cannot create a huge turn.
        if( Correction > mMaximumCorrection )
        {
            Correction = mMaximumCorrection;
        }
        else if( Correction < -mMaximumCorrection )
        {
            Correction = -mMaximumCorrection;
        }

        // Positive error turns right; negative error turns left.
        arLeftSpeed =
            mBaseSpeed + Correction;

        arRightSpeed =
            mBaseSpeed - Correction;
    }
}
