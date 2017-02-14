/**
 * This file is part of MSGD.
 *
 * Copyright (C) 2015-2017 Chao Gao <cg500 at cam dot ac dot uk> (University of Cambridge)
 * For more information see <https://github.com/chaogao-cam/MSGD>
 *
 * MSGD is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MSGD is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MSGD. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __MSGD__Poses__
#define __MSGD__Poses__

#include <assert.h>
#include <vector>
#include <memory>
using namespace std;

#include "../MathLib/MathLibCommon.h"
#include "../MathLib/Macros.h"
#include "../MathLib/Vector.h"
#include "../MathLib/Matrix.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

class Poses
{
protected:

public:
    Vector ids;
    Vector xs;
    Vector ys;
    Vector headings;
    int numPoses; // actual number of the poses, different from the length of all the vector containers
    
    inline Poses
        ()
    {
        init();
    }
    
    inline virtual ~Poses
        ()
    {
        Release();
    }
    
    inline virtual void Release
        ()
    {
    }
    
    // append a new pose to the end
    inline void addPose
        (REALTYPE poseID, REALTYPE x, REALTYPE y, REALTYPE heading)
    {
        // if container is not large enough, enlarge them to twice of their sizes
        if(xs.Size() == numPoses)
        {
            ids.Resize(numPoses*2,true);
            xs.Resize(numPoses*2,true);
            ys.Resize(numPoses*2,true);
            headings.Resize(numPoses*2,true);
        }

        ids[numPoses] = poseID;
        xs[numPoses] = x;
        ys[numPoses] = y;
        headings[numPoses] = heading;
        
        numPoses += 1;
    }
    
    inline void init
    ()
    {
        int initSize = 500;
        ids = Vector(initSize);
        xs = Vector(initSize);
        ys = Vector(initSize);
        headings = Vector(initSize);
        
        numPoses = 0;
    }
    
    inline void clear
    ()
    {
        init();
    }
    
    inline int findPoseIdxByID(int poseID)
    {
        for (int i = 0; i < numPoses; i++)
        {
            if(ids[i] == poseID)
            {
                return i;
            }
        }
        return -1;
    }
    
    void Print
        (string name = "");
};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif


#endif /* defined(__MSGD__Poses__) */
