//-----------------------------------------------------------------------------
// CRobot.h
//
// Declares the differential-drive robot used by both follower types. The robot
// stores its pose and trail, performs wheel-based motion, draws itself through
// CRender, and contains the A5 random position, heading and wheel-travel noise.
//-----------------------------------------------------------------------------

#ifndef CROBOT_H
#define CROBOT_H

#include "CLoopReader.h"
#include "CRender.h"

#include <vector>

class CRobot
{
    public:

        // Creates one robot from a map start pose and gives it its trail colour.
        CRobot(
            CRender& arRender,
            const CPose& arStartPose,
            Color aTrailColour );

        // Advances the robot one fixed simulation step using noisy wheel travel.
        void Update(
            float aLeftSpeed,
            float aRightSpeed,
            float aTimeStep );

        // Retained from A2 for collision detection; A5 does not count collisions.
        void CheckCollision(
            const CLoopReader& arLoop );

        // Draws the stored trail, robot body and heading indicator.
        void Draw() const;

        // Provides the pose needed by the wall or line controller.
        const CPose& GetPose() const;

        // Retained from A2; not used for A5 scoring because collisions are ignored.
        int GetCollisionCount() const;

    private:

        // Distance helper used by collision detection.
        float DistanceToSegment(
            Vec2D aPoint,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd ) const;

        // Produces a random value in [-aMaximumMagnitude, +aMaximumMagnitude].
        float RandomOffset(
            float aMaximumMagnitude ) const;

        // Current simulated position and orientation of this robot.
        CPose mPose;

        // Full trajectory, retained for the whole run as required by the lab.
        std::vector<Vec2D> mTrail;

        Color mTrailColour;

        // A2 collision state is kept even though A5 does not count collisions.
        int mCollisionCount;
        bool mWasColliding;

        // Physical robot dimensions.
        const float mRadius;
        const float mWheelSeparation;

        // A5 noise magnitudes. Each robot receives independent random samples.
        const float mStartPositionNoise;
        const float mStartHeadingNoise;
        const float mWheelNoise;

        // Non-owning renderer reference used for all robot drawing.
        CRender& mrRender;
};

#endif
