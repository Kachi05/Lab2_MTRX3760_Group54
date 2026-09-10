#include "CSimulation.h"

#include <iostream>
#include <vector>

//-----------------------------------------------------------------------------
CSimulation::CSimulation()
    :
        mpRobot( NULL ),
        mUpdateCount( 0 ),
        mTimeStep( 0.02f ),
        mMaximumUpdates( 4000 ),
        mWallThickness( 2.0f )
{
}

//-----------------------------------------------------------------------------
CSimulation::~CSimulation()
{
    delete mpRobot;

    mRender.CloseWindow();
}

//-----------------------------------------------------------------------------
bool CSimulation::Initialise()
{
    bool Okay =
        mWalls.ReadFile(
            "SimpleWalls.map" );

    if( Okay )
    {
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
        // This leaves the completed trail visible.
        Draw();
    }

    PrintSummary();
}

//-----------------------------------------------------------------------------
void CSimulation::Update()
{
    float LeftSpeed = 0.0f;
    float RightSpeed = 0.0f;

    mWallFollower.GetWheelSpeeds(
        mpRobot->GetPose(),
        mWalls,
        LeftSpeed,
        RightSpeed );

    mpRobot->Update(
        LeftSpeed,
        RightSpeed,
        mTimeStep );

    mpRobot->CheckCollision(
        mWalls );

    ++mUpdateCount;
}

//-----------------------------------------------------------------------------
void CSimulation::Draw()
{
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

    std::cout
        << "Updates: "
        << mUpdateCount
        << std::endl;

    std::cout
        << "Collisions: "
        << mpRobot->GetCollisionCount()
        << std::endl;
}
