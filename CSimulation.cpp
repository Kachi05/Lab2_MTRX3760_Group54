//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// Implements the A5 noise-bonus simulation. It creates 20 wall followers and
// 20 line followers, updates them independently using the existing A2 sensor
// controllers, and renders the resulting spread of noisy trajectories.
//-----------------------------------------------------------------------------

#include "CSimulation.h"

#include <iostream>
#include <vector>

//-----------------------------------------------------------------------------
CSimulation::CSimulation()
    :
        mUpdateCount( 0 ),
        mRobotsPerType( 20 ),       // A5 requires 20 of each robot type.
        mTimeStep( 0.02f ),         // Fixed simulated time per update.
        mMaximumUpdates( 4000 ),    // Long enough for the robots to traverse the loops.
        mWallThickness( 2.0f ),
        mLineThickness( 5.0f )      // Required floor line sensing width is 5 units.
{
}


//-----------------------------------------------------------------------------
CSimulation::~CSimulation()
{
    // Delete every wall follower created by Initialise().
    for( CRobot* pRobot : mWallRobots )
    {
        delete pRobot;
    }

    // Delete every line follower created by Initialise().
    for( CRobot* pRobot : mLineRobots )
    {
        delete pRobot;
    }

    // CSimulation owns the render window, so it closes it during cleanup.
    mRender.CloseWindow();
}


//-----------------------------------------------------------------------------
bool CSimulation::Initialise()
{
    // Load the room walls and the floor-line loop before creating any robots.
    const bool WallsOkay =
        mWalls.ReadFile(
            "SimpleWalls.map" );

    const bool LineOkay =
        mLine.ReadFile(
            "SimpleLine.map" );

    const bool Okay =
        WallsOkay && LineOkay;

    if( Okay )
    {
        // A5 requirement: create 20 noisy wall-following robots simultaneously.
        for( int i=0; i<mRobotsPerType; ++i )
        {
            mWallRobots.push_back(
                new CRobot(
                    mRender,
                    mWalls.GetStartPose(),
                    GREEN ) );
        }

        // A5 requirement: create 20 noisy line-following robots simultaneously.
        for( int i=0; i<mRobotsPerType; ++i )
        {
            mLineRobots.push_back(
                new CRobot(
                    mRender,
                    mLine.GetStartPose(),
                    RED ) );
        }
    }

    return Okay;
}


//-----------------------------------------------------------------------------
void CSimulation::Run()
{
    while( !mRender.WindowShouldClose() )
    {
        // Advance physics only until the requested simulation length is reached.
        if( mUpdateCount < mMaximumUpdates )
        {
            Update();
        }

        // Keep drawing after updates stop so the final spread can be screenshotted.
        Draw();
    }

    PrintSummary();
}


//-----------------------------------------------------------------------------
void CSimulation::Update()
{
    //---Wall-following robots---
    for( CRobot* pRobot : mWallRobots )
    {
        float LeftSpeed = 0.0f;
        float RightSpeed = 0.0f;

        // Controller reads this individual robot's noisy pose and senses the walls.
        mWallFollower.GetWheelSpeeds(
            pRobot->GetPose(),
            mWalls,
            LeftSpeed,
            RightSpeed );

        // CRobot applies new independent wheel-travel noise during this update.
        pRobot->Update(
            LeftSpeed,
            RightSpeed,
            mTimeStep );
    }


    //---Line-following robots---
    for( CRobot* pRobot : mLineRobots )
    {
        float LeftSpeed = 0.0f;
        float RightSpeed = 0.0f;

        // Each line robot senses the line using its own perturbed position/heading.
        mLineFollower.GetWheelSpeeds(
            pRobot->GetPose(),
            mLine,
            LeftSpeed,
            RightSpeed );

        pRobot->Update(
            LeftSpeed,
            RightSpeed,
            mTimeStep );
    }


    // A5 explicitly states that collisions under noise are not counted, so the
    // A2 CheckCollision() calls are intentionally omitted from this simulation.

    ++mUpdateCount;
}


//-----------------------------------------------------------------------------
void CSimulation::Draw()
{
    mRender.BeginDrawing();

    // Draw the shared environment before drawing all 40 robots and trails.
    DrawWalls();
    DrawLine();

    for( CRobot* pRobot : mWallRobots )
    {
        pRobot->Draw();
    }

    for( CRobot* pRobot : mLineRobots )
    {
        pRobot->Draw();
    }

    mRender.EndDrawing();
}


//-----------------------------------------------------------------------------
void CSimulation::DrawWalls()
{
    const std::vector<Vec2D>& Vertices =
        mWalls.GetVertices();

    if( !Vertices.empty() )
    {
        // Start from the final vertex so the loop closes back to the first one.
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
void CSimulation::DrawLine()
{
    const std::vector<Vec2D>& Vertices =
        mLine.GetVertices();

    if( !Vertices.empty() )
    {
        // The floor line is a closed loop just like the room-wall loop.
        Vec2D Previous =
            Vertices.back();

        for( const Vec2D& Vertex : Vertices )
        {
            mRender.DrawLine(
                Previous,
                Vertex,
                mLineThickness,
                YELLOW );

            Previous = Vertex;
        }
    }
}


//-----------------------------------------------------------------------------
void CSimulation::PrintSummary() const
{
    // Collisions are deliberately omitted because A5 says they are not counted.
    std::cout << std::endl;

    std::cout
        << "A5 noise simulation complete"
        << std::endl;

    std::cout
        << "Updates: "
        << mUpdateCount
        << std::endl;

    std::cout
        << "Wall-following robots: "
        << mWallRobots.size()
        << std::endl;

    std::cout
        << "Line-following robots: "
        << mLineRobots.size()
        << std::endl;
}
