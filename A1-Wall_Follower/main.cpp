#include "CSimulation.h"

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
