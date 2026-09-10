//-----------------------------------------------------------------------------
// CLineFollower.cpp
//
// Implements the line-following controller. The four possible combinations of
// the two sensor readings are used to drive straight or steer back toward the
// line. The controller itself remains deterministic; A5 noise is applied by
// CRobot to the physical wheel movement.
//-----------------------------------------------------------------------------

#include "CLineFollower.h"


//-----------------------------------------------------------------------------
CLineFollower::CLineFollower()
    :
        // Main sensor sits in front of the robot; side sensor is offset right.
        mMainSensor( 12.0f, 0.0f ),
        mSideSensor( 10.0f, 8.0f ),

        // Wheel speeds used by the line-following rules.
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
    // Sample the floor underneath both line sensors.
    const bool MainOnLine =
        mMainSensor.Sense(
            arRobotPose,
            arLoop );

    const bool SideOnLine =
        mSideSensor.Sense(
            arRobotPose,
            arLoop );


    // Ideal position: main sensor is on the line and the side sensor is off it.
    if( MainOnLine && !SideOnLine )
    {
        arLeftSpeed = mBaseSpeed;
        arRightSpeed = mBaseSpeed;
    }


    // Both sensors on the line means the line has shifted toward the right side.
    else if( MainOnLine && SideOnLine )
    {
        // A small speed difference gives a gentle right correction.
        arLeftSpeed = mFastSpeed;
        arRightSpeed = mBaseSpeed;
    }


    // Only the side sensor sees the line, so turn right more strongly.
    else if( !MainOnLine && SideOnLine )
    {
        arLeftSpeed = mFastSpeed;
        arRightSpeed = mSlowSpeed;
    }


    // Neither sensor sees the line, so search back toward the left.
    else
    {
        arLeftSpeed = mSlowSpeed;
        arRightSpeed = mFastSpeed;
    }
}
