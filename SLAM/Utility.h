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

#ifndef __MSGD__Utility__
#define __MSGD__Utility__

#include <assert.h>
#include <vector>
using namespace std;

#include "../MathLib/MathLibCommon.h"
#include "../MathLib/Macros.h"
#include "../MathLib/Vector.h"
#include "../MathLib/Matrix.h"
#include "Constraint.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

    inline void computeRotAndTransFromPose
    (Vector &t, Matrix &rot, Matrix &trans)
{
    REALTYPE theta = t.RefNoCheck(2);
    REALTYPE s=sin(theta), c=cos(theta);
    
    rot.RefNoCheck(0,0)=c, rot.RefNoCheck(0,1)=-s;
    rot.RefNoCheck(1,0)=s, rot.RefNoCheck(1,1)= c;
    
    trans.RefNoCheck(0,0)=c, trans.RefNoCheck(0,1)=-s, trans.RefNoCheck(0,2)=t.RefNoCheck(0);
    trans.RefNoCheck(1,0)=s, trans.RefNoCheck(1,1)=c, trans.RefNoCheck(1,2)=t.RefNoCheck(1);
}

inline void computeFullTransFromPose
    (Vector &t, Matrix &trans)
{
    REALTYPE theta = t.RefNoCheck(2);
    REALTYPE s=sin(theta), c=cos(theta);
    
    trans.RefNoCheck(0,0)=c, trans.RefNoCheck(0,1)=-s, trans.RefNoCheck(0,2)=t.RefNoCheck(0);
    trans.RefNoCheck(1,0)=s, trans.RefNoCheck(1,1)=c, trans.RefNoCheck(1,2)=t.RefNoCheck(1);
    trans.RefNoCheck(2,0)=0, trans.RefNoCheck(2,1)=0, trans.RefNoCheck(2,2)=1;
}


inline void computePoseFromTrans
    (Matrix &rot, Vector &pose)
{
    pose.RefNoCheck(0) = rot.RefNoCheck(0, 2);
    pose.RefNoCheck(1) = rot.RefNoCheck(1, 2);
    pose.RefNoCheck(2) = atan2(rot.RefNoCheck(1,0),rot.RefNoCheck(0,0));
}

inline void computeFullRotFromPose
    (Vector &pose, Matrix &rot)
{
    REALTYPE theta = pose.RefNoCheck(2);
    REALTYPE s=sin(theta), c=cos(theta);
    
    rot.RefNoCheck(0,0)=c, rot.RefNoCheck(0,1)=-s, rot.RefNoCheck(0,2)=0;
    rot.RefNoCheck(1,0)=s, rot.RefNoCheck(1,1)=c, rot.RefNoCheck(1,2)=0;
    rot.RefNoCheck(2,0)=0, rot.RefNoCheck(2,1)=0, rot.RefNoCheck(2,2)=1;
}

inline void computeRotFromPose
    (Vector &pose, Matrix &rot)
{
    REALTYPE theta = pose.RefNoCheck(2);
    REALTYPE s=sin(theta), c=cos(theta);
    
    rot.RefNoCheck(0,0)=c, rot.RefNoCheck(0,1)=-s;
    rot.RefNoCheck(1,0)=s, rot.RefNoCheck(1,1)= c;
}
    
inline int myRandom
    (int i)
{
    return rand()%i;
}

inline REALTYPE normalizeAngle(REALTYPE angle)
{
    return atan2(sin(angle),cos(angle));
}
    
#ifdef USE_MATHLIB_NAMESPACE
}
#endif


#endif /* defined(__MSGD__Utility__) */
