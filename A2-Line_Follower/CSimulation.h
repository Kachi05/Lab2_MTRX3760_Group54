//-----------------------------------------------------------------------------
// CSimulation.h
//
// Declares the simulation used to run the wall-following and line-following
// robots together. It manages the maps, robots, controllers and renderer.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H
#include "CLoopReader.h"
#include "CRender.h"
#include "CRobot.h"
#include "CWallFollower.h"
#include "CLineFollower.h"

//-----------------------------------------------------------------------------
// CSimulation manages the maps, robots, controllers and renderer needed
// to run both wall-following and line-following simulations.
//-----------------------------------------------------------------------------
class CSimulation
{
    public:

        CSimulation();
        ~CSimulation();

        // Loads both maps and creates the two robots.
        bool Initialise();

        // Runs the simulation until the render window is closed.
        void Run();

    private:

        // Advances both robots by one simulation timestep.
        void Update();

        // Draws the current simulation state.
        void Draw();

        // Draws the room walls.
        void DrawWalls();

        // Draws the floor line.
        void DrawLine();

        // Reports the final update and collision counts.
        void PrintSummary() const;

        //---Renderer---
        CRender mRender;

        //---Maps---#ifndef CSIMULATION_H
#define CSIMULATION_H

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