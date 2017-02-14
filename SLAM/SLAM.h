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

#ifndef __MSGD__SLAM__
#define __MSGD__SLAM__

#include <assert.h>
#include <vector>
#include <set>
#include <fstream>
#include <sstream>
#include <sys/time.h>
#include <ctime>
#include <string>
#include <limits>

using namespace std;

#include "../MathLib/MathLibCommon.h"
#include "MiniBatch.h"
#include "SLAMParas.h"
#include "LogTree.h"
#include "Poses.h"
#include "Constraints.h"
#include "Utility.h"


#ifdef USE_MATHLIB_NAMESPACE
namespace MathLib {
#endif

class SLAM
{
public:

    SLAMParas mParameters; // paras
    Poses mPoses; // system states
    Constraints mConstraints; // all constraints
    LogTree mDTree; // the tree hold changes to poses
    LogTree mCWTree; // the tree used to compute cumulative weights
    int iteration;
    
    // for learning rate computation
    REALTYPE gammaX,gammaY,gammaH;
    double alpha[3];
    
    // constructor
    inline SLAM
        ()
    {
        cout << endl <<
        "MSGD Copyright (C) 2015-2017 Chao Gao, University of Cambridge." << endl <<
        "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
        "This is free software, and you are welcome to redistribute it" << endl <<
        "under certain conditions. See LICENSE.txt." << endl << endl;

        init();
    }
    
    // Destructor
    inline virtual ~SLAM
        ()
    {
        Release();
    }
    
    inline virtual void Release
        ()
    {
    }
    
    // load g2o file
    void loadData
        ();
    
    inline void clear
    ()
    {
        iteration = 0;
        gammaX = 0;
        gammaY = 0;
        gammaH = 0;
        mPoses.clear();
        mConstraints.clear();
        mDTree.clear();
        mCWTree.clear();
    }
    
    // init the system
    inline void init
        ()
    {
        iteration = 0;
        gammaX = 0;
        gammaY = 0;
        gammaH = 0;
        mDTree.init(mPoses.numPoses);
        mCWTree.init(mPoses.numPoses);
        mCWTree.setEqualCumulativeWeights();
        
        if (mParameters.enableRandom)
        {
            // seed the random number generator
            std::srand (unsigned(std::time(0)));
        }
    }
    
    // graph optimisation
    inline void optimise
    ()
    {
        saveGraph("initial");
        
        // show initial error
        REALTYPE globalError = computeGlobalChi2Error();
        cout << "Before Optimization: " << " global error = " << globalError << "   error/constraint = " << globalError/mConstraints.numCsts << endl;
        
        cout << "**** Optimization Start ****" << endl;
        struct timeval ts, te;
        gettimeofday(&ts,0);
        for(iteration = 1; iteration <= mParameters.iterations; iteration++)
        {
            iterate();
            
            if(mParameters.saveAllIters)
            {
                std::ostringstream graphName;
                graphName << "_iter" << iteration;
                saveGraph(graphName.str());
            }
            
            if(mParameters.showProgress)
            {
                // show error
                REALTYPE error = computeGlobalChi2Error();
                cout << "Iteration " << iteration << ": global error = " << error << "   error/constraint = " << error/mConstraints.numCsts << endl;
            }
        }
        
        gettimeofday(&te,0);
        cout << "**** Optimization Done ****" << endl;
        
        // compute the error and dump it
        globalError = computeGlobalChi2Error();
        cout << "After Optimization: " << " global error = " << globalError << "   error/constraint = " << globalError/mConstraints.numCsts << endl;
        
        double dts=(te.tv_sec-ts.tv_sec)+1e-6*(te.tv_usec-ts.tv_usec);
        cout << "Total Time = " << dts << " s." << endl;
        
        saveGraph("final");
    }
    
    void Print
        (string name = "");
    
    void saveGraph
        (string name = "");
        
protected:
    
    // get current pose value
    inline void getPose
    (int idx, Vector &pose)
    {
        mDTree.get(idx, pose);
        
        pose.RefNoCheck(0) += mPoses.xs.RefNoCheck(idx);
        pose.RefNoCheck(1) += mPoses.ys.RefNoCheck(idx);
        pose.RefNoCheck(2) += mPoses.headings.RefNoCheck(idx);
    }
    
    inline vector<Constraint *>::iterator findMaxErrConstraint(MiniBatch &miniBatch)
    {
        vector<Constraint *>::iterator itMaxCstP = miniBatch.constraints.begin();
        Constraint *maxCstP = *itMaxCstP;
        computeResidualAndW(maxCstP);
        
        if (miniBatch.size > 1)
        {
            // Here we use squared error to approximate chi2 error
            // which is faster, but not very rigorous because the three components (x,y,h) have different units of measure and need to be scaled properly by W.
            // But, this does not make big difference on the converged results as tested.
            // Another possible implementation is to find a max error constraint for each component of r.
            
            // REALTYPE maxErr = computeChi2Err(maxCstP);
            REALTYPE maxErr = computeSquaredErr(maxCstP);
            
            for (vector<Constraint *>::iterator itCstP = itMaxCstP+1; itCstP != miniBatch.constraints.end(); itCstP++)
            {
                // assume the W of all the constraints within a mini-batch are similar
                (*itCstP)->W = maxCstP->W;
                computeResidual(*itCstP);
                
                // REALTYPE err = computeChi2Err(*itCstP);
                REALTYPE err = computeSquaredErr(*itCstP);
                
                if(err > maxErr)
                {
                    itMaxCstP = itCstP;
                    maxErr = err;
                }
            }
        }
        
        return itMaxCstP;
    }
    
    inline REALTYPE computeGlobalChi2Error
    ()
    {
        REALTYPE globalError=0.;
        
        for (vector<Constraint* >::iterator itCstP = mConstraints.constraints.begin(); itCstP != mConstraints.constraints.end(); itCstP++)
        {
            computeResidualAndW(*itCstP);
            globalError += computeChi2Err(*itCstP);
        }
        
        return globalError;
    }
    
    inline void computeResidualAndW(Constraint *cst)
    {
        Matrix tempResult(3,3);
        
        // compute residual
        computeResidual(cst);
        
        // compute W
        // W = rotA*cst->info*rotA.Transpose();
        cst->rotA.Mult(cst->info,tempResult);
        tempResult.Mult(cst->rotA.Transpose(),cst->W);
    }
    
    inline void computeResidual(Constraint *cst)
    {
        Vector poseA(3),poseB(3),poseAB(3);
        Matrix transAB(3,3);
        
        getPose(cst->aIdx, poseA);
        getPose(cst->bIdx, poseB);
        
        computeRotAndTransFromPose(poseA, cst->rotA, cst->transA);
        cst->transA.Mult(cst->trans, transAB);
        computePoseFromTrans(transAB, poseAB);
        
        // compute residual r
        cst->r.RefNoCheck(0) = poseAB.RefNoCheck(0) - poseB.RefNoCheck(0);
        cst->r.RefNoCheck(1) = poseAB.RefNoCheck(1) - poseB.RefNoCheck(1);
        cst->r.RefNoCheck(2) = poseAB.RefNoCheck(2) - poseB.RefNoCheck(2);
        
        // normalize angle
        cst->r.RefNoCheck(2) = normalizeAngle(cst->r.RefNoCheck(2));
    }
    
    inline REALTYPE computeChi2Err(Constraint *cst)
    {
        Vector r1(3);
        Matrix tempResult(3,3);
        
        r1 = cst->W*cst->r;
        REALTYPE chi2Error = cst->r.RefNoCheck(0)*r1.RefNoCheck(0) + cst->r.RefNoCheck(1)*r1.RefNoCheck(1) + cst->r.RefNoCheck(2)*r1.RefNoCheck(2);
        
        // store chi2 error
        cst->chi2Error = chi2Error;
        
        return chi2Error;
    }
    
    // approximate Chi2Err by squared error: r[0]*r[0]+r[1]*r[1]+r[2]*r[2]
    inline REALTYPE computeSquaredErr(Constraint *cst)
    {
        REALTYPE errX = cst->r.RefNoCheck(0);
        REALTYPE errY = cst->r.RefNoCheck(1);
        REALTYPE errH = cst->r.RefNoCheck(2);
        
        REALTYPE squaredError = errX*errX + errY*errY + errH*errH;
        
        return squaredError;
    }
    
    inline void iterate
    ()
    {
        //recompute M only at iterations 1, 2, 4, 8, and so on.
        double logItrs = std::log(iteration)/std::log(2);
        if( logItrs == ceil(logItrs) )
        {
            if (iteration > 1)
            {
                updatePoses();
            }
            
            computeH();
        }
        
        updateAlpha();
        
        distributeErrors();
        
    }
    
    inline void updateAlpha()
    {
        alpha[0] = 1. / (gammaX*iteration);
        alpha[1] = 1. / (gammaY*iteration);
        alpha[2] = 1. / (gammaH*iteration);
    }
    
    // update the estimated system states
    inline void updatePoses
    ()
    {
        // compute all the changes to the poses
        Vector deltas(mPoses.numPoses*3);
        mDTree.computeAllValues(deltas);
        
        // update the system states by these changes
        for (int i = 0; i < mPoses.numPoses; i++)
        {
            int di = i*3;
            mPoses.xs.RefNoCheck(i) += deltas.RefNoCheck(di);
            mPoses.ys.RefNoCheck(i) += deltas.RefNoCheck(di+1);
            mPoses.headings.RefNoCheck(i) += deltas.RefNoCheck(di+2);
        }
        
        // reset Dtree
        mDTree.reset();
    }
    
    // compute the preconditioner H
    inline void computeH
    ()
    {
        gammaX = gammaY = gammaH = numeric_limits<double>::max();
        mCWTree.reset(); // the tree used to compute cumulative weights
        
        Vector poseA(3);
        Matrix W(3,3), tempResult(3,3);
        REALTYPE w00 = 0.0, w11 = 0.0, w22 = 0.0;
        
        bool WComputed; // assume the W for constraints in a mini-batch are the same
        
        for(vector<MiniBatch>::iterator itMiniBatch = mConstraints.miniBatches.begin(); itMiniBatch != mConstraints.miniBatches.end(); itMiniBatch++)
        {
            WComputed = false;
            MiniBatch &miniBatch = *itMiniBatch;
            
            for (vector<Constraint *>::iterator itCstP = miniBatch.constraints.begin(); itCstP != miniBatch.constraints.end(); itCstP++)
            {
                Constraint *cst = *itCstP;
                
                if(!WComputed)
                {
                    // compute W
                    poseA.RefNoCheck(0) = mPoses.xs.RefNoCheck(cst->aIdx);
                    poseA.RefNoCheck(1) = mPoses.ys.RefNoCheck(cst->aIdx);
                    poseA.RefNoCheck(2) = mPoses.headings.RefNoCheck(cst->aIdx);
                    
                    computeRotFromPose(poseA, cst->rotA);
                    
                    // W = rotA*cst->info*rotA.Transpose();
                    cst->rotA.Mult(cst->info,tempResult);
                    tempResult.Mult(cst->rotA.Transpose(),W);
                    
                    w00=W.RefNoCheck(0, 0);
                    w11=W.RefNoCheck(1, 1);
                    w22=W.RefNoCheck(2, 2);
                    
                    gammaX = gammaX < w00 ? gammaX : w00;
                    gammaY = gammaY < w11 ? gammaY : w11;
                    gammaH = gammaH < w22 ? gammaH : w22;
                    
                    WComputed = true;
                }
                
                int num = cst->bIdx - cst->aIdx;
                assert(num > 1); // no relative motion constraint
                mCWTree.distributeAmount(cst->aIdx+1, cst->bIdx, w00*num, w11*num, w22*num);
            }
        }
        
        // set cumulative weights
        mDTree.setCW(mCWTree);
    }
    
    // distribute errors
    inline void distributeErrors
    ()
    {
        if(mParameters.enableRandom)
        {
            mConstraints.shuffleMiniBatches();
        }
        
        vector<MiniBatch>::iterator itMiniBatchBegin = mConstraints.miniBatches.begin();
        vector<MiniBatch>::iterator itMiniBatch;
        
        for(vector<int>::iterator itMiniBatchIdx = mConstraints.miniBatchesIdxArray.begin(); itMiniBatchIdx != mConstraints.miniBatchesIdxArray.end(); itMiniBatchIdx++)
        {
            int iMiniBatch = *itMiniBatchIdx;
            itMiniBatch = itMiniBatchBegin + iMiniBatch;
            
            MiniBatch &miniBatch = *itMiniBatch;
            vector<Constraint *>::iterator itMaxCstP = findMaxErrConstraint(miniBatch);
            
            // solve the constraint with the max error
            Constraint *maxErrCst = *itMaxCstP;
            solveConstraint(maxErrCst);
            
            if (miniBatch.size == 1)
            {
                continue;
            }
            
            double weightForMaxErrCst[3];
            mDTree.getTotalWeight(maxErrCst->aIdx+1, maxErrCst->bIdx, weightForMaxErrCst);
            
            vector<Constraint *>::iterator itCstP;
            if (miniBatch.type == MiniBatch::AIncBInc)
            {
                // solve each component separately
                bool xSolved = false;
                bool ySolved = false;
                bool hSolved = false;
                
                // loop from first cst to max error cst, once a consistent cst found, solve it and break
                for (itCstP = miniBatch.constraints.begin(); itCstP != itMaxCstP;itCstP++)
                {
                    Constraint *cst = *itCstP;
                    
                    // compute residual 1
                    double residual_1[3];
                    for (int i = 0; i < 3; i++)
                    {
                        if((cst->r.RefNoCheck(i) * maxErrCst->r.RefNoCheck(i)) < 0)
                        {
                            // not consistent
                            residual_1[i] = 0;
                            continue;
                        }
                        
                        double overlappedResidual; // residual overlapped with max err cst
                        
                        if(cst->bIdx > maxErrCst->aIdx)
                        {
                            double overlappedWeight = mDTree.getTotalWeight(maxErrCst->aIdx+1, cst->bIdx, i);
                            
                            overlappedResidual = maxErrCst->r.RefNoCheck(i) * overlappedWeight/weightForMaxErrCst[i];
                        }
                        else
                        {
                            // no ovelap with max err cst
                            overlappedResidual = 0;
                        }
                        
                        residual_1[i] = cst->r.RefNoCheck(i) - overlappedResidual;
                        
                        if (i == 2)
                        {
                            residual_1[i] = normalizeAngle(residual_1[i]);
                        }
                        
                        if((residual_1[i]*cst->r.RefNoCheck(i)) < 0)
                        {
                            // not consistent
                            residual_1[i] = 0;
                        }
                    }
                    
                    if (xSolved) residual_1[0] = 0;
                    
                    if (ySolved) residual_1[1] = 0;
                    
                    if (hSolved) residual_1[2] = 0;
                    
                    if((residual_1[0] == 0) && (residual_1[1] == 0) && (residual_1[2] == 0))
                    {
                        // no need to solve
                        continue;
                    }
                    
                    if(residual_1[0] != 0) xSolved = true;
                    
                    if(residual_1[1] != 0) ySolved = true;
                    
                    if(residual_1[2] != 0) hSolved = true;
                    
                    // compute residual 2
                    double residual_2[3];
                    double weightResidual[3];
                    double cstTotalWeight[3];
                    mDTree.getTotalWeight(cst->aIdx+1, maxErrCst->aIdx, weightResidual);
                    mDTree.getTotalWeight(cst->aIdx+1, cst->bIdx, cstTotalWeight);
                    residual_2[0] = cst->r.RefNoCheck(0) * weightResidual[0] / cstTotalWeight[0];
                    residual_2[1] = cst->r.RefNoCheck(1) * weightResidual[1] / cstTotalWeight[1];
                    residual_2[2] = cst->r.RefNoCheck(2) * weightResidual[2] / cstTotalWeight[2];
                    
                    // compute residual
                    Vector residual(3);
                    residual.RefNoCheck(0) = (fabs(residual_1[0])>fabs(residual_2[0])) ? residual_2[0] : residual_1[0];
                    residual.RefNoCheck(1) = (fabs(residual_1[1])>fabs(residual_2[1])) ? residual_2[1] : residual_1[1];
                    residual.RefNoCheck(2) = (fabs(residual_1[2])>fabs(residual_2[2])) ? residual_2[2] : residual_1[2];
                    
                    Vector d = cst->W*residual*2.;
                    
                    double beta[3];
                    double l = maxErrCst->aIdx - cst->aIdx;
                    
                    beta[0] = l*alpha[0]*d.RefNoCheck(0);
                    beta[1] = l*alpha[1]*d.RefNoCheck(1);
                    beta[2] = l*alpha[2]*d.RefNoCheck(2);
                    
                    REALTYPE errX = residual.RefNoCheck(0);
                    REALTYPE errY = residual.RefNoCheck(1);
                    REALTYPE errH = residual.RefNoCheck(2);
                    
                    beta[0]=(fabs(beta[0])>fabs(errX)) ? errX : beta[0];
                    beta[1]=(fabs(beta[1])>fabs(errY)) ? errY : beta[1];
                    beta[2]=(fabs(beta[2])>fabs(errH)) ? errH : beta[2];
                    
                    mDTree.distributeAmount(cst->aIdx+1, maxErrCst->aIdx, beta[0], beta[1], beta[2]);
                    
                    if(xSolved && ySolved && hSolved)
                    {
                        break;
                    }
                }
                
                // loop from the last cst to the max error cst, once a consistent cst found, solve it and break
                xSolved = false;
                ySolved = false;
                hSolved = false;
                
                for (itCstP = miniBatch.constraints.end(); itCstP != itMaxCstP+1;)
                {
                    --itCstP;
                    
                    Constraint *cst = *itCstP;
                    
                    // compute residual 1
                    double residual_1[3];
                    for (int i = 0; i < 3; i++)
                    {
                        if((cst->r.RefNoCheck(i) * maxErrCst->r.RefNoCheck(i)) < 0)
                        {
                            // not consistent
                            residual_1[i] = 0;
                            continue;
                        }
                        
                        double overlappedResidual; // residual overlapped with max err cst
                        
                        if(cst->aIdx < maxErrCst->bIdx)
                        {
                            double overlappedWeight = mDTree.getTotalWeight(cst->aIdx+1, maxErrCst->bIdx, i);
                            overlappedResidual = maxErrCst->r.RefNoCheck(i) * overlappedWeight/weightForMaxErrCst[i];
                        }
                        else
                        {
                            // no ovelap with max err cst
                            overlappedResidual = 0;
                        }
                        
                        residual_1[i] = cst->r.RefNoCheck(i) - overlappedResidual;
                        
                        if (i == 2)
                        {
                            residual_1[i] = normalizeAngle(residual_1[i]);
                        }
                        
                        if((residual_1[i]*cst->r.RefNoCheck(i)) < 0)
                        {
                            // not consistent
                            residual_1[i] = 0;
                        }
                    }
                    
                    if (xSolved) residual_1[0] = 0;
                    
                    if (ySolved) residual_1[1] = 0;
                    
                    if (hSolved) residual_1[2] = 0;
                    
                    if((residual_1[0] == 0) && (residual_1[1] == 0) && (residual_1[2] == 0))
                    {
                        // no need to solve
                        continue;
                    }
                    
                    if(residual_1[0] != 0) xSolved = true;
                    
                    if(residual_1[1] != 0) ySolved = true;
                    
                    if(residual_1[2] != 0) hSolved = true;
                    
                    // compute residual 2
                    double residual_2[3];
                    double weightResidual[3];
                    double cstTotalWeight[3];
                    mDTree.getTotalWeight(maxErrCst->bIdx+1, cst->bIdx, weightResidual);
                    mDTree.getTotalWeight(cst->aIdx+1, cst->bIdx, cstTotalWeight);
                    residual_2[0] = cst->r.RefNoCheck(0) * weightResidual[0] / cstTotalWeight[0];
                    residual_2[1] = cst->r.RefNoCheck(1) * weightResidual[1] / cstTotalWeight[1];
                    residual_2[2] = cst->r.RefNoCheck(2) * weightResidual[2] / cstTotalWeight[2];
                    
                    // compute residual
                    Vector residual(3);
                    residual.RefNoCheck(0) = (fabs(residual_1[0])>fabs(residual_2[0])) ? residual_2[0] : residual_1[0];
                    residual.RefNoCheck(1) = (fabs(residual_1[1])>fabs(residual_2[1])) ? residual_2[1] : residual_1[1];
                    residual.RefNoCheck(2) = (fabs(residual_1[2])>fabs(residual_2[2])) ? residual_2[2] : residual_1[2];
                    
                    Vector d = cst->W*residual*2.;
                    
                    double beta[3];
                    double l = cst->bIdx - maxErrCst->bIdx;
                    
                    beta[0] = l*alpha[0]*d.RefNoCheck(0);
                    beta[1] = l*alpha[1]*d.RefNoCheck(1);
                    beta[2] = l*alpha[2]*d.RefNoCheck(2);
                    
                    REALTYPE errX = residual.RefNoCheck(0);
                    REALTYPE errY = residual.RefNoCheck(1);
                    REALTYPE errH = residual.RefNoCheck(2);
                    
                    beta[0]=(fabs(beta[0])>fabs(errX)) ? errX : beta[0];
                    beta[1]=(fabs(beta[1])>fabs(errY)) ? errY : beta[1];
                    beta[2]=(fabs(beta[2])>fabs(errH)) ? errH : beta[2];
                    
                    mDTree.distributeAmount(maxErrCst->bIdx+1, cst->bIdx, beta[0], beta[1], beta[2]);
                    
                    if(xSolved && ySolved && hSolved)
                    {
                        break;
                    }
                }
                
            }
            else if (miniBatch.type == MiniBatch::AIncBDec)
            {
                cout << "miniBatch.type == MiniBatch::AIncBDec" << endl;
                assert(false); // TODO
                
                // loop from first cst to max error cst, once a consistent cst found, solve it and break
                for (itCstP = itMaxCstP; itCstP != miniBatch.constraints.begin();)
                {
                    --itCstP;
                    Constraint *cst = *itCstP;
                }
            }
            else
            {
                cerr << "Unknown mini-batch type." << endl;
                assert(false);
            }
        }
    }
    
    inline void solveConstraint(Constraint *cst)
    {
        Vector d = cst->W*cst->r*2.;
        
        double l = cst->bIdx - cst->aIdx;
        
        double beta[3];
        
        beta[0] = l*alpha[0]*d.RefNoCheck(0);
        beta[1] = l*alpha[1]*d.RefNoCheck(1);
        beta[2] = l*alpha[2]*d.RefNoCheck(2);
        
        REALTYPE errX = cst->r.RefNoCheck(0);
        REALTYPE errY = cst->r.RefNoCheck(1);
        REALTYPE errH = cst->r.RefNoCheck(2);
        
        beta[0]=(fabs(beta[0])>fabs(errX)) ? errX : beta[0];
        beta[1]=(fabs(beta[1])>fabs(errY)) ? errY : beta[1];
        beta[2]=(fabs(beta[2])>fabs(errH)) ? errH : beta[2];
        
        mDTree.distributeAmount(cst->aIdx+1, cst->bIdx, beta[0], beta[1], beta[2]);
    }
};

#ifdef USE_MATHLIB_NAMESPACE
}
#endif


#endif /* defined(__MSGD__SLAM__) */
