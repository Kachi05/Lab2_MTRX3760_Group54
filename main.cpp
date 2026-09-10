//-----------------------------------------------------------------------------
// main.cpp
//
// Entry point for the A5 noisy robot simulation. It seeds the pseudo-random
// number generator so each run receives a different set of perturbations, then
// creates, initialises and runs the single CSimulation object.
//-----------------------------------------------------------------------------

#include "CSimulation.h"

#include <cstdlib>
#include <ctime>

int main()
{
    // Seed rand() once so the robots follow different noisy paths on each run.
    std::srand(
        static_cast<unsigned int>(
            std::time( NULL ) ) );

    // CSimulation owns and coordinates all other simulation objects.
    CSimulation Simulation;

    // Stop cleanly if either map file cannot be loaded.
    if( !Simulation.Initialise() )
    {
        return 1;
    }

    // Run fixed updates and rendering until the user closes the window.
    Simulation.Run();

    return 0;
}
