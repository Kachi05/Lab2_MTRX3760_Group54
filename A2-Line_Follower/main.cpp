//-----------------------------------------------------------------------------
// main.cpp
//
// Entry point for the MTRX3760 Lab 2 A2 robot simulation.
// The simulation is created, initialised and then run until the render window
// is closed.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#include "CSimulation.h"
//-----------------------------------------------------------------------------
// Creates, initialises and runs the complete A2 robot simulation.
//-----------------------------------------------------------------------------
int main()
{
    CSimulation Simulation;

    if( !Simulation.Initialise() )
    {
        return 1;
    }

    Simulation.Run();

    return 0;
}