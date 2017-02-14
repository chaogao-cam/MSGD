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

#ifndef __MSGD__WTree__
#define __MSGD__WTree__

#include <assert.h>
#include <vector>
#include <queue>
using namespace std;

#include "../MathLib/MathLibCommon.h"
#include "../MathLib/Macros.h"
#include "../MathLib/Vector.h"
#include "../MathLib/Matrix.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
#define SET_BIT(var,pos) ((var) | (1<<(pos)))

#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif
    
    class LogTree
    {
    public:
        
        Vector offmuls; // array for offx,mulx,offy,muly,offheading,mulheading
        Matrix CW; // cumulative weights, Nx3
        int MAXIDX;
        
        vector<int> bfsNodeIdx; // the node indices in breadth first search order
        vector<int> fatherIdx; // the fater indices of the nodes in bfsNodeIdx

        inline LogTree
        ()
        {
            init();
        }

        inline LogTree
        (int size)
        {
            init(size);
        }

        // set cumulative weights
        inline void setCW
        (LogTree &cwTree)
        {
            assert(cwTree.MAXIDX == MAXIDX);
            
            Vector treeValues(MAXIDX*3);
            cwTree.computeAllValues(treeValues);
            
            double Mi[3];
            
            CW.RefNoCheck(0,0) = ((treeValues.RefNoCheck(0) == 0)?0:1./treeValues.RefNoCheck(0));
            CW.RefNoCheck(0,1) = ((treeValues.RefNoCheck(1) == 0)?0:1./treeValues.RefNoCheck(1));
            CW.RefNoCheck(0,2) = ((treeValues.RefNoCheck(2) == 0)?0:1./treeValues.RefNoCheck(2));

            for (int i = 1; i < MAXIDX; i++)
            {
                int lastIdx = (i-1) * 3;
                int idx = i * 3;

                Mi[0] = treeValues.RefNoCheck(idx) - treeValues.RefNoCheck(lastIdx);
                Mi[1] = treeValues.RefNoCheck(idx+1) - treeValues.RefNoCheck(lastIdx+1);
                Mi[2] = treeValues.RefNoCheck(idx+2) - treeValues.RefNoCheck(lastIdx+2);

                Mi[0] = (Mi[0]==0)?0:1./Mi[0];
                Mi[1] = (Mi[1]==0)?0:1./Mi[1];
                Mi[2] = (Mi[2]==0)?0:1./Mi[2];
                
                CW.RefNoCheck(i,0) = CW.RefNoCheck(i-1,0) + Mi[0];
                CW.RefNoCheck(i,1) = CW.RefNoCheck(i-1,1) + Mi[1];
                CW.RefNoCheck(i,2) = CW.RefNoCheck(i-1,2) + Mi[2];
            }
        }
        
        // for sppeeding up preconditioner computation
        // the motivation is that the use of LogTree requires to
        // compute comulative weights (CW) from M, i.e. CW(i) = sum M from 0 to i
        // so we can use the same tree structure to speed up this process
        // and here every node has the same weight (set to 1)
        inline void setEqualCumulativeWeights
        ()
        {
            assert(CW.RowSize() == MAXIDX);
            
            // every node has equal weight, i.e. 1
            for (int i = 0; i < MAXIDX; i++)
            {
                CW.RefNoCheck(i,0) = i;
                CW.RefNoCheck(i,1) = i;
                CW.RefNoCheck(i,2) = i;
            }
        }
        
        // compute all values in O(n) time using the breadth first search order
        inline void computeAllValues
        (Vector &values)
        {
            assert(values.Size() == MAXIDX*3);
            
            Vector cumulativeOffmuls = Vector(MAXIDX*6); // store the intermediate cumulative off muls
            
            REALTYPE wX,wY,wH; // self's weights
            
            // store the off and mul of node 0 (the root)
            cumulativeOffmuls.RefNoCheck(0) = offmuls.RefNoCheck(0);
            cumulativeOffmuls.RefNoCheck(1) = offmuls.RefNoCheck(1);
            cumulativeOffmuls.RefNoCheck(2) = offmuls.RefNoCheck(2);
            cumulativeOffmuls.RefNoCheck(3) = offmuls.RefNoCheck(3);
            cumulativeOffmuls.RefNoCheck(4) = offmuls.RefNoCheck(4);
            cumulativeOffmuls.RefNoCheck(5) = offmuls.RefNoCheck(5);
            
            // compute the value of node 0 (the root)
            wX=CW.RefNoCheck(0,0), wY = CW.RefNoCheck(0,1), wH = CW.RefNoCheck(0,2);
            values.RefNoCheck(0) = offmuls.RefNoCheck(0) + offmuls.RefNoCheck(1) * wX;
            values.RefNoCheck(1) = offmuls.RefNoCheck(2) + offmuls.RefNoCheck(3) * wY;
            values.RefNoCheck(2) = offmuls.RefNoCheck(4) + offmuls.RefNoCheck(5) * wH;

            // compute the values of other nodes in the breadth first search order
            for (int i = 1; i < MAXIDX; i++)
            {
                int idx = bfsNodeIdx[i];
                int idxBase = idx*6;
                int vIdxBase = idx * 3;
                
                int fIdx = fatherIdx[i];
                int fIdxBase = fIdx * 6;
                
                // compute (self+parent)'s off and mul
                cumulativeOffmuls.RefNoCheck(idxBase) = cumulativeOffmuls.RefNoCheck(fIdxBase) + offmuls.RefNoCheck(idxBase);
                cumulativeOffmuls.RefNoCheck(idxBase+1) = cumulativeOffmuls.RefNoCheck(fIdxBase+1) + offmuls.RefNoCheck(idxBase+1);
                cumulativeOffmuls.RefNoCheck(idxBase+2) = cumulativeOffmuls.RefNoCheck(fIdxBase+2) + offmuls.RefNoCheck(idxBase+2);
                cumulativeOffmuls.RefNoCheck(idxBase+3) = cumulativeOffmuls.RefNoCheck(fIdxBase+3) + offmuls.RefNoCheck(idxBase+3);
                cumulativeOffmuls.RefNoCheck(idxBase+4) = cumulativeOffmuls.RefNoCheck(fIdxBase+4) + offmuls.RefNoCheck(idxBase+4);
                cumulativeOffmuls.RefNoCheck(idxBase+5) = cumulativeOffmuls.RefNoCheck(fIdxBase+5) + offmuls.RefNoCheck(idxBase+5);
                
                // compute self's value
                wX=CW.RefNoCheck(idx,0), wY = CW.RefNoCheck(idx,1), wH = CW.RefNoCheck(idx,2);
                values.RefNoCheck(vIdxBase) = cumulativeOffmuls.RefNoCheck(idxBase) + cumulativeOffmuls.RefNoCheck(idxBase+1)*wX;
                values.RefNoCheck(vIdxBase+1) = cumulativeOffmuls.RefNoCheck(idxBase+2) + cumulativeOffmuls.RefNoCheck(idxBase+3)*wY;
                values.RefNoCheck(vIdxBase+2) = cumulativeOffmuls.RefNoCheck(idxBase+4) + cumulativeOffmuls.RefNoCheck(idxBase+5)*wH;
            }
        }

        inline virtual ~LogTree
        ()
        {
            Release();
        }
        
        inline virtual void Release
        ()
        {
        }
        
        inline void reset()
        {
            offmuls.Zero();
        }
        
        inline void init()
        {
            offmuls = Vector(0);
            CW = Matrix(0,0);
            MAXIDX = 0;
            bfsNodeIdx.clear();
            fatherIdx.clear();
        }

        inline void init
        (int size)
        {
            if(size == 0)
            {
                init();
                return;
            }
            
            offmuls = Vector(size*6);
            CW = Matrix(size,3);
            MAXIDX = size;
            bfsNodeIdx.clear();
            fatherIdx.clear();
            breadthFirstSearch();
        }
        
        inline void clear()
        {
            init();
        }
        
        // Distribute "amount" between buckets [a,b]
        // scaled using cumulative weights
        inline void distributeAmount(int a, int b, REALTYPE xAmt, REALTYPE yAmt, REALTYPE hAmt)
        {
            if(a > b)
            {
                return;
            }
            
            REALTYPE totalweightX = CW.RefNoCheck(b, 0) - CW.RefNoCheck(a-1,0);
            REALTYPE totalweightY = CW.RefNoCheck(b, 1) - CW.RefNoCheck(a-1,1);
            REALTYPE totalweightH = CW.RefNoCheck(b, 2) - CW.RefNoCheck(a-1,2);
         
            REALTYPE mulX = xAmt/totalweightX;
            REALTYPE mulY = yAmt/totalweightY;
            REALTYPE mulH = hAmt/totalweightH;
            
            REALTYPE offX = CW.RefNoCheck(a-1, 0) * mulX;
            REALTYPE offY = CW.RefNoCheck(a-1, 1) * mulY;
            REALTYPE offH = CW.RefNoCheck(a-1, 2) * mulH;
            
            add(a, 
                -offX, mulX,
                -offY, mulY,
                -offH, mulH);
            
            add(b+1, 
                xAmt+offX, -mulX,
                yAmt+offY, -mulY,
                hAmt+offH, -mulH);
        }
        
        inline void getTotalWeight(int a, int b, double totalweight[])
        {
            totalweight[0] = CW.RefNoCheck(b, 0) - CW.RefNoCheck(a-1,0);
            totalweight[1] = CW.RefNoCheck(b, 1) - CW.RefNoCheck(a-1,1);
            totalweight[2] = CW.RefNoCheck(b, 2) - CW.RefNoCheck(a-1,2);
        }
        
        inline double getTotalWeight(int a, int b, int index)
        {
            return CW.RefNoCheck(b, index) - CW.RefNoCheck(a-1,index);
        }
        
        /** Get the current value of the specified index. Runs in O(log N) time. **/
        void get(int idx, Vector &delta)
        {
            REALTYPE deltaX=0, deltaY=0, deltaH=0;
            REALTYPE wX=CW.RefNoCheck(idx,0), wY = CW.RefNoCheck(idx,1), wH = CW.RefNoCheck(idx,2);

            while (true)
            {
                int didx = idx*6;
                deltaX += offmuls.RefNoCheck(didx + 0);
                deltaX += offmuls.RefNoCheck(didx + 1) * wX;
                deltaY += offmuls.RefNoCheck(didx + 2);
                deltaY += offmuls.RefNoCheck(didx + 3) * wY;
                deltaH += offmuls.RefNoCheck(didx + 4);
                deltaH += offmuls.RefNoCheck(didx + 5) * wH;
                
                if (idx == 0)
                {
                    break;
                }
                
                idx = idx&(idx-1);
            }

            delta.RefNoCheck(0) = deltaX;
            delta.RefNoCheck(1) = deltaY;
            delta.RefNoCheck(2) = deltaH;
        }

        void Print
        (string name = "");
        
    protected:
        /** Inserts increments into the tree. (There are actually two
         trees, one for off and mul, but we always access both at the
         same time, so we do the operations on both simultaneously. The
         array d[] contains both trees, interleaved.
         **/
        inline void add(int idx,
                        REALTYPE offX, REALTYPE mulX,
                        REALTYPE offY, REALTYPE mulY,
                        REALTYPE offH, REALTYPE mulH)
        {
            while (idx < MAXIDX)
            {
                int didx = idx*6;
                offmuls.RefNoCheck(didx + 0) += offX;
                offmuls.RefNoCheck(didx + 1) += mulX;
                offmuls.RefNoCheck(didx + 2) += offY;
                offmuls.RefNoCheck(didx + 3) += mulY;
                offmuls.RefNoCheck(didx + 4) += offH;
                offmuls.RefNoCheck(didx + 5) += mulH;
                
                if (idx == 0)
                {
                    break;
                }
                
                idx = idx + ((idx&(idx-1))^idx);
            }
        }
        
        // compute the breadth first search order and store them in bfsNodeIdx
        // fatherIdx stores the father node indices of the nodes in bfsNodeIdx
        inline void breadthFirstSearch()
        {
            if(MAXIDX == 0)
            {
                bfsNodeIdx.clear();
                fatherIdx.clear();
                return;
            }
            
            queue<int> nodeQ;
            
            nodeQ.push(0); // push the root
            
            bfsNodeIdx.push_back(0);
            fatherIdx.push_back(0);
            
            // the limit of the children number of any node (not a strict limit)
            int ChildrenNumberLimit = maxChildrenNumber();
            int *childrenIdxSet = new int[ChildrenNumberLimit];
            int numOfChildren;
            
            while (!nodeQ.empty())
            {
                int idx = nodeQ.front();
                nodeQ.pop();
                
                // push the children of the node idx into the queue
                numOfChildren = getChildrenIdxSet(idx, childrenIdxSet);
                
                for (int i = 0; i < numOfChildren; i++)
                {
                    int childIdx = childrenIdxSet[i];
                    bfsNodeIdx.push_back(childIdx);
                    fatherIdx.push_back(idx);
                    
                    nodeQ.push(childIdx);
                }
            }
            
            delete [] childrenIdxSet;
        }

        // return the max number of children a node could have
        inline int maxChildrenNumber()
        {
            // the number of '0's at the end of the binary form of a node's index number is the number of its children
            // the children number of the node with MAXIDX is an upper limit for all nodes
            // so we just return the number of binary bits of MAXIDX as a not so strict bound
            if(MAXIDX == 0)
            {
                return 0;
            }
            
            return ceil(std::log(MAXIDX)/std::log(2));
        }
        
        inline int getChildrenIdxSet(int idx, int *childrenIdxSet)
        {
            // check from right to left if a bit is zero
            // if so, set this bit to 1 and this is a child index
            // stop when find a bit of 1
            int numOfChildren = 0;
            
            while(CHECK_BIT(idx,numOfChildren) == 0)
            {
                int childIdx = SET_BIT(idx,numOfChildren);
                if(childIdx < MAXIDX)
                {
                    childrenIdxSet[numOfChildren] = childIdx;
                    numOfChildren++;
                }
                else
                {
                    break;
                }
            }
            
            return numOfChildren;
        }
    };
    
#ifdef USE_MATHLIB_NAMESPACE
}
#endif

#endif /* defined(__MSGD__WTree__) */
