//-----------------------------------------------------------------------------
// CRobot.h
//
// Declares the differential-drive robot used for both A2 simulations.
// The robot stores its pose, travelled path and collision count and provides
// functions for movement, collision checking and rendering.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#ifndef CROBOT_H
#define CROBOT_H

#include "CLoopReader.h"
#include "CRender.h"

#include <vector>

//-----------------------------------------------------------------------------
// CRobot models the differential-drive robot used by both followers.
// It stores the robot's pose, travelled path and collision information.
//-----------------------------------------------------------------------------
class CRobot
{
    public:

        // Creates the robot at the given pose and sets its trail colour.
        CRobot(
            CRender& arRender,
            const CPose& arStartPose,
            Color aTrailColour );

        // Updates the robot pose using the left and right wheel speeds.
        void Update(
            float aLeftSpeed,
            float aRightSpeed,
            float aTimeStep );

        // Checks whether the robot is colliding with any wall in the loop.
        void CheckCollision(
            const CLoopReader& arLoop );

        // Draws the robot, heading and travelled path.
        void Draw() const;

        // Returns the robot's current pose.
        const CPose& GetPose() const;

        // Returns the number of separate collision events.
        int GetCollisionCount() const;

    private:

        // Finds the shortest distance from a point to a wall segment.
        float DistanceToSegment(
            Vec2D aPoint,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd ) const;

        //---Robot state---
        CPose mPose;

        //---Trail---
        std::vector<Vec2D> mTrail;
        Color mTrailColour;

        //---Collision state---
        int mCollisionCount;
        bool mWasColliding;

        //---Robot dimensions---
        const float mRadius;
        const float mWheelSeparation;

        //---Renderer---
        CRender& mrRender;
};

#endif