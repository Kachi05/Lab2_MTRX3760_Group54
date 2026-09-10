//-----------------------------------------------------------------------------
// main.cpp
//
// Entry point for the A1 wall-following program. It creates the simulation,
// initialises the supplied wall map and robot, then starts the simulation loop.
//-----------------------------------------------------------------------------

#include "CSimulation.h"

int main()
{
    // This object owns and coordinates the complete robot simulation.
    CSimulation Simulation;

    // Stop immediately if the wall map cannot be loaded or setup fails.
    if( !Simulation.Initialise() )
    {
        return 1;
    }

    // Run until the user closes the simulation window.
    Simulation.Run();

    return 0;
}
