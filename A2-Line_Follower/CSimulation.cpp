#include "CSimulation.h"

#include <iostream>
#include <vector>

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
CSimulation::~CSimulation()
{
    delete mpWallRobot;
    delete mpLineRobot;

    mRender.CloseWindow();
}


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

        mpLineRobot = new CRobot(
            mRender,
            mLine.GetStartPose(),
            RED );
            }

    return Okay;
}


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