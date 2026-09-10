//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// Implements the A1 simulation loop. It loads SimpleWalls.map, creates the
// wall-following robot, repeatedly senses/updates/checks collisions using a
// fixed timestep, renders the result, and prints the final simulation summary.
//-----------------------------------------------------------------------------

#include "CSimulation.h"

#include <iostream>
#include <vector>

//-----------------------------------------------------------------------------
CSimulation::CSimulation()
    :
        mpRobot( NULL ),
        mUpdateCount( 0 ),

        // A fixed timestep is used rather than real elapsed frame time.
        mTimeStep( 0.02f ),

        // Run long enough for the robot to complete the supplied room circuit.
        mMaximumUpdates( 4000 ),
        mWallThickness( 2.0f )
{
}

//-----------------------------------------------------------------------------
CSimulation::~CSimulation()
{
    // The simulation owns the dynamically created robot.
    delete mpRobot;

    // Shut down the raylib window through the supplied renderer wrapper.
    mRender.CloseWindow();
}

//-----------------------------------------------------------------------------
bool CSimulation::Initialise()
{
    // Load the room walls and starting pose from the supplied map file.
    bool Okay =
        mWalls.ReadFile(
            "SimpleWalls.map" );

    if( Okay )
    {
        // Create the robot only after the map has loaded successfully.
        mpRobot = new CRobot(
            mRender,
            mWalls.GetStartPose() );
    }

    return Okay;
}

//-----------------------------------------------------------------------------
void CSimulation::Run()
{
    while( !mRender.WindowShouldClose() )
    {
        // Stop physics after the required
        // number of simulation updates.
        if( mUpdateCount < mMaximumUpdates )
        {
            Update();
        }

        // Keep drawing even after movement stops.
        // This leaves the completed trail visible for the final screenshot.
        Draw();
    }

    // Print the required report information once the window is closed.
    PrintSummary();
}

//-----------------------------------------------------------------------------
void CSimulation::Update()
{
    float LeftSpeed = 0.0f;
    float RightSpeed = 0.0f;

    // Use the two range sensors to calculate the wheel command.
    mWallFollower.GetWheelSpeeds(
        mpRobot->GetPose(),
        mWalls,
        LeftSpeed,
        RightSpeed );

    // Advance the robot exactly one fixed simulation timestep.
    mpRobot->Update(
        LeftSpeed,
        RightSpeed,
        mTimeStep );

    // Check and report whether the new robot position contacts a wall.
    mpRobot->CheckCollision(
        mWalls );

    ++mUpdateCount;
}

//-----------------------------------------------------------------------------
void CSimulation::Draw()
{
    // All raylib drawing is performed through CRender.
    mRender.BeginDrawing();

    DrawWalls();

    mpRobot->Draw();

    mRender.EndDrawing();
}

//-----------------------------------------------------------------------------
void CSimulation::DrawWalls()
{
    const std::vector<Vec2D>& Vertices =
        mWalls.GetVertices();

    if( !Vertices.empty() )
    {
        // Start from the last vertex so the first drawn segment closes the loop.
        Vec2D Previous =
            Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            mRender.DrawLine(
                Previous,
                Vertex,
                mWallThickness,
                RAYWHITE );

            Previous = Vertex;
        }
    }
}

//-----------------------------------------------------------------------------
void CSimulation::PrintSummary() const
{
    std::cout << std::endl;

    std::cout
        << "Simulation complete"
        << std::endl;

    // A1 requires the total number of completed updates.
    std::cout
        << "Updates: "
        << mUpdateCount
        << std::endl;

    // A1 also requires the final collision count.
    std::cout
        << "Collisions: "
        << mpRobot->GetCollisionCount()
        << std::endl;
}
