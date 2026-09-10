//-----------------------------------------------------------------------------
// CSimulation.h
//
// Declares the owner/coordinator for the complete A5 simulation. It loads both
// maps, owns the controllers and renderer, creates 20 robots of each type, runs
// every robot on the same fixed timestep, and draws all noisy trajectories.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CLoopReader.h"
#include "CRender.h"
#include "CRobot.h"
#include "CWallFollower.h"
#include "CLineFollower.h"

#include <vector>

class CSimulation
{
    public:

        // Sets simulation constants; robot objects are created in Initialise().
        CSimulation();

        // Releases all dynamically created robots and closes the render window.
        ~CSimulation();

        // Loads both map files and creates the 20 wall + 20 line robots.
        bool Initialise();

        // Runs fixed updates until the limit, while continuing to display trails.
        void Run();

    private:

        // Advances all 40 robots by one fixed simulation timestep.
        void Update();

        // Draws the room, floor line and every robot/trail.
        void Draw();

        // Drawing helpers for the two closed loops loaded from the map files.
        void DrawWalls();
        void DrawLine();

        // Prints a concise A5 end-of-run summary.
        void PrintSummary() const;

        //---Renderer---
        CRender mRender;

        //---Maps---
        CLoopReader mWalls;
        CLoopReader mLine;

        //---A5 robots---
        // Each vector owns 20 dynamically created CRobot objects.
        std::vector<CRobot*> mWallRobots;
        std::vector<CRobot*> mLineRobots;

        //---Controllers---
        // One deterministic controller can be reused by every robot of its type.
        CWallFollower mWallFollower;
        CLineFollower mLineFollower;

        //---Simulation---
        int mUpdateCount;

        const int mRobotsPerType;
        const float mTimeStep;
        const int mMaximumUpdates;

        //---Drawing---
        const float mWallThickness;
        const float mLineThickness;
};

#endif
