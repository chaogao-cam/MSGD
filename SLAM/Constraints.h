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

#ifndef __MSGD__Constraints__
#define __MSGD__Constraints__

#include <assert.h>
#include <vector>
#include <algorithm>
using namespace std;

#include "../MathLib/MathLibCommon.h"
#include "../MathLib/Macros.h"
#include "../MathLib/Vector.h"
#include "../MathLib/Matrix.h"
#include "Utility.h"
#include "Constraint.h"
#include "MiniBatch.h"

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif
    inline bool cstCompare
    (const Constraint *cst1, const Constraint* cst2)
    {
        // sort by aIdx first and then bIdx
        if(cst1->aIdx == cst2->aIdx)
        {
            return cst1->bIdx < cst2->bIdx;
        }
        else
        {
            return cst1->aIdx < cst2->aIdx;
        }
    }
    
    class Constraints
    {
    public:
        vector<Constraint *> constraints;
        vector<int> cstsIdxArray;
        int numCsts;
        
        vector<MiniBatch> miniBatches;
        vector<int> miniBatchesIdxArray;
        int numMiniBatches;
        
        inline Constraints
        ()
        {
        }
        
        inline virtual ~Constraints
        ()
        {
            Release();
        }
        
        inline virtual void Release
        ()
        {
        }
        
        inline void shuffleConstraints()
        {
            std::random_shuffle(cstsIdxArray.begin(), cstsIdxArray.end(), myRandom);
        }

        inline void shuffleMiniBatches()
        {
            std::random_shuffle(miniBatchesIdxArray.begin(), miniBatchesIdxArray.end(), myRandom);
        }

        inline Constraint *addConstraint
        (int aID, int bID,int aIdx,int bIdx, Vector &t, Matrix &info)
        {
            Constraint *constraint = new Constraint;
            
            assert(aIdx < bIdx);
            
            constraint->aIdx = aIdx;
            constraint->bIdx = bIdx;
            constraint->aID = aID;
            constraint->bID = bID;
            constraint->t = t;
            constraint->info = info;
            
            computeFullRotFromPose(t, constraint->rot);
            computeFullTransFromPose(t, constraint->trans);
            
            insertCstSorted(constraint); // constraints.push_back(cst);
            cstsIdxArray.push_back((int)cstsIdxArray.size());
            numCsts = (int)constraints.size();
            
            return constraint;
        }
        
        inline void addMiniBatch
        (vector<Constraint *> &csts)
        {
            MiniBatch miniBatch(csts);
            determineMiniBatchType(miniBatch);
            miniBatches.push_back(miniBatch);
            miniBatchesIdxArray.push_back((int)miniBatchesIdxArray.size());
            numMiniBatches = (int)miniBatches.size();
        }
        
        inline void clear()
        {
            // clear constraints
            for(int i = 0; i < constraints.size(); i++)
            {
                delete constraints[i];
            }
            constraints.clear();
            
            cstsIdxArray.clear();
            numCsts = 0;
            
            // clear mini-batches
            miniBatches.clear();
            miniBatchesIdxArray.clear();
            numMiniBatches = 0;
        }
        
        void Print
        (string name = "");
    protected:
        
        // determin the type of this mini-batch
        inline void determineMiniBatchType(MiniBatch &miniBatch)
        {
            assert(miniBatch.size > 1);
            
            // Because the constraints have been sorted by aIdx and bIdx
            // we only need to look at the b to determine the type of the mini-batch
            const Constraint *firstCst = miniBatch.constraints[0];
            const Constraint *secondCst = miniBatch.constraints[1];
            
            assert(firstCst->bIdx != secondCst->bIdx);
            if (firstCst->bIdx > secondCst->bIdx)
            {
                miniBatch.type = MiniBatch::AIncBDec;
            }
            else if (firstCst->bIdx < secondCst->bIdx)
            {
                miniBatch.type = MiniBatch::AIncBInc;
            }
            else
            {
                cerr << "B should be strictly monotically increasing/decreasing.";
                assert(false);
            }
            
        }

        // keep constraints sorted in ascending order using (aIdx, bIdx)
        inline void insertCstSorted(Constraint *cst)
        {
            vector<Constraint *>::iterator it = std::lower_bound(constraints.begin(), constraints.end(), cst, cstCompare);
            constraints.insert(it, cst);
        }
    };
    
#ifdef USE_MATHLIB_NAMESPACE
}
#endif


#endif /* defined(__MSGD__Constraints__) */
