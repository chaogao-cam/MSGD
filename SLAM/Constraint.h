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

#ifndef __MSGD__Constraint__
#define __MSGD__Constraint__

#include <assert.h>
#include <vector>
using namespace std;

#include "../MathLib/MathLibCommon.h"
#include "../MathLib/Macros.h"
#include "../MathLib/Vector.h"
#include "../MathLib/Matrix.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif
    
    class Constraint
    {
    protected:
        
    public:
        
        // aIdx < bIdx
        int aIdx;
        int bIdx;
        
        // aID < bID
        int aID;
        int bID;

        Vector r; // the vector of (rX,rY,rHeading)
        Matrix W; // W = rotA*cst.info*rotA.Transpose();

        Matrix rotA; // rotation matrix of pose A
        Matrix transA; // transformation matrix of pose A

        REALTYPE chi2Error;
        
        Vector t;
        Matrix info;
        Matrix rot;
        Matrix trans;
        
        inline Constraint
        ()
        {
            t = Vector(3);
            info = Matrix(3,3);
            rot = Matrix(3,3);
            trans = Matrix(3,3);
            r = Vector(3);
            W = Matrix(3,3);
            
            rotA = Matrix(3,3);
            transA = Matrix(3,3);
            
            // fill in part of the matrices
            rotA.RefNoCheck(0,2) = 0, rotA.RefNoCheck(1,2) = 0;
            rotA.RefNoCheck(2,0)=0, rotA.RefNoCheck(2,1)= 0, rotA.RefNoCheck(2,2) = 1;
            transA.RefNoCheck(2,0)=0, transA.RefNoCheck(2,1)=0, transA.RefNoCheck(2,2)=1;
        }

        inline virtual ~Constraint
        ()
        {
            Release();
        }
        
        inline virtual void Release
        ()
        {
        }

        void Print
        (string name = "") const;
    };
    
#ifdef USE_MATHLIB_NAMESPACE
}
#endif


#endif /* defined(__MSGD__Constraint__) */
