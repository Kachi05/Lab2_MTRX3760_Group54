#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CLoopReader.h"
#include "CRender.h"
#include "CRobot.h"
#include "CWallFollower.h"
#include "CLineFollower.h"

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
        void DrawLine();

        void PrintSummary() const;

        //---Renderer---
        CRender mRender;

        //---Maps---
        CLoopReader mWalls;
        CLoopReader mLine;

        //---Robots---
        CRobot* mpWallRobot;
        CRobot* mpLineRobot;

        //---Controllers---
        CWallFollower mWallFollower;
        CLineFollower mLineFollower;

        //---Simulation---
        int mUpdateCount;

        const float mTimeStep;
        const int mMaximumUpdates;

        //---Drawing---
        const float mWallThickness;
        const float mLineThickness;
};

#endif