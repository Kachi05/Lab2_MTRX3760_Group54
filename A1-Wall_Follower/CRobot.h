#ifndef CROBOT_H
#define CROBOT_H

#include "CLoopReader.h"
#include "CRender.h"

#include <vector>

class CRobot
{
    public:

        CRobot(
            CRender& arRender,
            const CPose& arStartPose );

        void Update(
            float aLeftSpeed,
            float aRightSpeed,
            float aTimeStep );

        void CheckCollision(
            const CLoopReader& arLoop );

        void Draw() const;

        const CPose& GetPose() const;

        int GetCollisionCount() const;

    private:

        float DistanceToSegment(
            Vec2D aPoint,
            Vec2D aSegmentStart,
            Vec2D aSegmentEnd ) const;

        CPose mPose;

        std::vector<Vec2D> mTrail;

        int mCollisionCount;
        bool mWasColliding;

        const float mRadius;
        const float mWheelSeparation;

        CRender& mrRender;
};

#endif
