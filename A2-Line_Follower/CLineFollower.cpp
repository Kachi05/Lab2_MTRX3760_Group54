#include "CLineFollower.h"


//-----------------------------------------------------------------------------
CLineFollower::CLineFollower()
    :
        mMainSensor( 12.0f, 0.0f ),
        mSideSensor( 10.0f, 8.0f ),

        mBaseSpeed( 22.0f ),
        mSlowSpeed( 0.0f ),
        mFastSpeed( 28.0f )
{
}


//-----------------------------------------------------------------------------
void CLineFollower::GetWheelSpeeds(
    const CPose& arRobotPose,
    const CLoopReader& arLoop,
    float& arLeftSpeed,
    float& arRightSpeed ) const
{
    const bool MainOnLine =
        mMainSensor.Sense(
            arRobotPose,
            arLoop );

    const bool SideOnLine =
        mSideSensor.Sense(
            arRobotPose,
            arLoop );


    // Main sensor is on the line and the
    // side sensor is off the line:
    // robot is in a good position.
    if( MainOnLine && !SideOnLine )
    {
        arLeftSpeed = mBaseSpeed;
        arRightSpeed = mBaseSpeed;
    }


    // Both sensors are on the line:
    // line has moved slightly towards
    // the right-hand sensor.
    else if( MainOnLine && SideOnLine )
    {
        // Gentle right turn.
        arLeftSpeed = mFastSpeed;
        arRightSpeed = mBaseSpeed;
    }


    // Only the right-side sensor sees
    // the line:
    // turn right more strongly.
    else if( !MainOnLine && SideOnLine )
    {
        arLeftSpeed = mFastSpeed;
        arRightSpeed = mSlowSpeed;
    }


    // Neither sensor sees the line:
    // search back towards the left.
    else
    {
        arLeftSpeed = mSlowSpeed;
        arRightSpeed = mFastSpeed;
    }
}