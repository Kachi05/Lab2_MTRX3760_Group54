//-----------------------------------------------------------------------------
// CSimulation.h
//
// Declares the object that owns and coordinates the complete A1 simulation.
// It loads the room, creates the robot, runs fixed-timestep updates, draws the
// scene, and prints the required end-of-run summary.
//-----------------------------------------------------------------------------

#ifndef CSIMULATION_H
#define CSIMULATION_H

#include "CLoopReader.h"
#include "CRender.h"
#include "CRobot.h"
#include "CWallFollower.h"

//-----------------------------------------------------------------------------
// Owns the main simulation objects and coordinates their interaction.
//-----------------------------------------------------------------------------
class CSimulation
{
    public:

        // Creates the simulation objects and sets fixed simulation constants.
        CSimulation();
        ~CSimulation();

        // Loads the map and creates the robot at its supplied start pose.
        bool Initialise();

        // Runs the simulation and rendering loop until the window is closed.
        void Run();

    private:

        // Advances the controller, robot motion, collision checking and counter.
        void Update();

        // Draws one frame of the room and robot.
        void Draw();

        // Draws the closed wall loop read from SimpleWalls.map.
        void DrawWalls();

        // Prints the required update and collision summary to the console.
        void PrintSummary() const;

        // Renderer and wall-map data owned by the simulation.
        CRender mRender;
        CLoopReader mWalls;

        // Robot is allocated after the map has been loaded successfully.
        CRobot* mpRobot;

        // Controller responsible for selecting the wheel speeds each update.
        CWallFollower mWallFollower;

        // Number of fixed simulation updates completed.
        int mUpdateCount;

        // Fixed timestep, run length, and wall drawing thickness.
        const float mTimeStep;
        const int mMaximumUpdates;
        const float mWallThickness;
};

#endif
