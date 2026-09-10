//-----------------------------------------------------------------------------
// CSimulation.cpp
//
// Implements the combined A2 robot simulation.
// Both robots are updated using the same fixed timestep and are drawn together
// with their maps, travelled paths and collision results.
//
// MTRX3760 Lab 2 - A2
//-----------------------------------------------------------------------------

#include "CSimulation.h"

#include <iostream>
#include <vector>

//-----------------------------------------------------------------------------
// Creates the simulation with its timestep and drawing settings.
//-----------------------------------------------------------------------------
CSimulation::CSimulation()
    :
        mpWallRobot( NULL ),
        mpLineRobot( NULL ),
        mUpdateCount( 0 ),
        mTimeStep( 0.02f ),
        mMaximumUpdates( 4000 ),
        mWallThickness( 2.0f ),
        mLineThickness( 5.0f )
{
}


//-----------------------------------------------------------------------------
// Deletes both robots and closes the render window.
//-----------------------------------------------------------------------------
CSimulation::~CSimulation()
{
    delete mpWallRobot;
    delete mpLineRobot;

    mRender.CloseWindow();
}


//-----------------------------------------------------------------------------
// Loads the wall and line maps and creates both robots at their start poses.
//-----------------------------------------------------------------------------
bool CSimulation::Initialise()
{
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
        // Wall-following robot starts from
        // the pose in SimpleWalls.map.
        mpWallRobot = new CRobot(
            mRender,
            mWalls.GetStartPose(),
            GREEN );

        // Line-following robot starts from
        // the pose in SimpleLine.map.
        mpLineRobot = new CRobot(
            mRender,
            mLine.GetStartPose(),
            RED );
    }

    return Okay;
}


//-----------------------------------------------------------------------------
// Repeatedly updates and draws both robots until the window is closed.
//-----------------------------------------------------------------------------
void CSimulation::Run()
{
    while( !mRender.WindowShouldClose() )
    {
        // Both robots advance once for every
        // fixed simulation update.
        if( mUpdateCount < mMaximumUpdates )
        {
            Update();
        }

        // Continue drawing after the simulation
        // stops so the completed trails remain visible.
        Draw();
    }

    PrintSummary();
}


//-----------------------------------------------------------------------------
// Advances both robots once and checks each for wall collisions.
//-----------------------------------------------------------------------------
void CSimulation::Update()
{
    //---Wall-following robot---

    float WallLeftSpeed = 0.0f;
    float WallRightSpeed = 0.0f;

    mWallFollower.GetWheelSpeeds(
        mpWallRobot->GetPose(),
        mWalls,
        WallLeftSpeed,
        WallRightSpeed );

    mpWallRobot->Update(
        WallLeftSpeed,
        WallRightSpeed,
        mTimeStep );


    //---Line-following robot---

    float LineLeftSpeed = 0.0f;
    float LineRightSpeed = 0.0f;

    mLineFollower.GetWheelSpeeds(
        mpLineRobot->GetPose(),
        mLine,
        LineLeftSpeed,
        LineRightSpeed );

    mpLineRobot->Update(
        LineLeftSpeed,
        LineRightSpeed,
        mTimeStep );


    //---Collision checking---

    // Both robots exist inside the room, so collisions
    // are checked against the room walls rather than
    // against the floor line.
    mpWallRobot->CheckCollision(
        mWalls );

    mpLineRobot->CheckCollision(
        mWalls );


    // One simulation update has now been completed
    // by both robots.
    ++mUpdateCount;
}


//-----------------------------------------------------------------------------
// Draws both maps and both robots in the same simulation window.
//-----------------------------------------------------------------------------
void CSimulation::Draw()
{
    mRender.BeginDrawing();

    DrawWalls();
    DrawLine();

    mpWallRobot->Draw();
    mpLineRobot->Draw();

    mRender.EndDrawing();
}


//-----------------------------------------------------------------------------
// Draws every segment of the closed room-wall loop.
//-----------------------------------------------------------------------------
void CSimulation::DrawWalls()
{
    const std::vector<Vec2D>& Vertices =
        mWalls.GetVertices();

    if( !Vertices.empty() )
    {
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
// Draws every segment of the closed floor-line loop.
//-----------------------------------------------------------------------------
void CSimulation::DrawLine()
{
    const std::vector<Vec2D>& Vertices =
        mLine.GetVertices();

    if( !Vertices.empty() )
    {
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
// Prints the completed update count and collision results for both robots.
//-----------------------------------------------------------------------------
void CSimulation::PrintSummary() const
{
    const int TotalCollisions =
        mpWallRobot->GetCollisionCount()
        + mpLineRobot->GetCollisionCount();

    std::cout << std::endl;

    std::cout
        << "Simulation complete"
        << std::endl;

    std::cout
        << "Updates: "
        << mUpdateCount
        << std::endl;

    std::cout
        << "Wall follower collisions: "
        << mpWallRobot->GetCollisionCount()
        << std::endl;

    std::cout
        << "Line follower collisions: "
        << mpLineRobot->GetCollisionCount()
        << std::endl;

    std::cout
        << "Total collisions: "
        << TotalCollisions
        << std::endl;
}