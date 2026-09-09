#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CLoopReader.h"
#include "CRender.h"
#include "CRobot.h"
#include "CWallFollower.h"

class CSimulation
{
    public:

        CSimulation();
        ~CSimulation();

        bool Initialise();
        void Run();

    private:

        void Update();
        void Draw();
        void DrawWalls();
        void PrintSummary() const;

        CRender mRender;
        CLoopReader mWalls;

        CRobot* mpRobot;

        CWallFollower mWallFollower;

        int mUpdateCount;

        const float mTimeStep;
        const int mMaximumUpdates;
        const float mWallThickness;
};

#endif
