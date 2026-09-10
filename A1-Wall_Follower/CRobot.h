//-----------------------------------------------------------------------------
// CRobot.h
//
// Declares the simulated differential-drive robot. The class stores the robot
// pose and trail, updates its motion from two wheel speeds, detects collisions,
// and draws the robot and its complete trajectory through CRender.
//-----------------------------------------------------------------------------

#ifndef CROBOT_H
#define CROBOT_H

#include "CLoopReader.h"
#include "CRender.h"

#include <vector>

//-----------------------------------------------------------------------------
// Represents the physical robot used by the wall-following simulation.
//-----------------------------------------------------------------------------
class CRobot
{
    public:

        // Creates the robot at the starting pose supplied by the map.
        CRobot(
            CRender& arRender,
            const CPose& arStartPose );

        // Advances the differential-drive motion by one fixed simulation step.
        void Update(
            float aLeftSpeed,
            float aRightSpeed,
            float aTimeStep );

        // Checks whether the circular robot overlaps any room wall.
        void CheckCollision(
            const CLoopReader& arLoop );

        // Draws the full trail, robot body, and heading indicator.
        void Draw() const;

        // Provides the current position and heading to the controller/sensors.
        const CPose& GetPose() const;

        // Returns the total number of separate collision events.
        int GetCollisionCount() const;

    private:

        // Finds the shortest distance from a point to one finite wall segment.
        float DistanceToSegment(
            Vec2D aPoint,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd ) const;

        // Current robot position and heading.
        CPose mPose;

        // Stores every visited position so the whole trajectory stays visible.
        std::vector<Vec2D> mTrail;

        // Collision state is used to count each contact event only once.
        int mCollisionCount;
        bool mWasColliding;

        // Physical dimensions of the simulated differential-drive robot.
        const float mRadius;
        const float mWheelSeparation;

        // Non-owning reference used for all robot drawing.
        CRender& mrRender;
};

#endif
