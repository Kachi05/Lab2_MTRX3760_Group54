#ifndef CLOOPREADER_H
#define CLOOPREADER_H

#include "CRender.h"   // for Vec2D

#include <string>
#include <vector>

//-----------------------------------------------------------------------------
// A pose: a position together with a heading, in radians.
//-----------------------------------------------------------------------------
struct CPose
{
    Vec2D mPosition;
    float mHeading;   // radians; 0 faces +x (right), PI/2 faces +y (down)
};

//-----------------------------------------------------------------------------
// CLoopReader: reads one loop file and stores the closed loop it describes.
//-----------------------------------------------------------------------------
class CLoopReader
{
    public:
        //---Ctor---
        CLoopReader();

        //---File reading---
        bool ReadFile( const std::string& arFilename );

        //---Access to the loop that was read---
        const std::string& GetName() const;
        const CPose& GetStartPose() const;
        const std::vector<Vec2D>& GetVertices() const;

    private:
        //---The loop---
        std::string mName;
        CPose mStartPose;
        std::vector<Vec2D> mVertices;   // the loop's corners; last joins to first
};

#endif
