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

#ifndef __MSGD__MiniBatch__
#define __MSGD__MiniBatch__

#include <assert.h>
#include <vector>
using namespace std;

#include "../MathLib/MathLibCommon.h"
#include "Constraint.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif
    
    class MiniBatch
    {
    protected:
        
    public:
        // define the type of a mini-batch (constraints have been sorted by a and then b)
        enum MiniBatchType {AIncBInc, AIncBDec};
        // AIncBInc are like:
        // 3 ........ 9
        //   4 ........ 10
        //     5 ........ 11
        //      6 ........ 12
        // where a increases monotonically by 0,1,2..., b increases strictly monotonically by 1,
        
        
        // AIncBDec are like:
        // 2 。........ 12
        //  3 ........ 11
        //   4.......10
        //    5.....9
        // where a increases monotonically by 0,1,2..., b decreases strictly monotonically by 1,
        
        vector<Constraint *> constraints; // the constraints in this mini-batch
        int size; // number of constraints
        MiniBatchType type; // the type of this mini-batch
        
        inline MiniBatch
        ()
        {
        }
        
        inline MiniBatch
        (vector<Constraint *> &csts):constraints(csts)
        {
            size = (int)csts.size();
        }
        
        inline virtual ~MiniBatch
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


#endif /* defined(__MSGD__MiniBatch__) */
