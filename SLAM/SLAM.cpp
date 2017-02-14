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

#include <memory>
#include <fstream>

#include "SLAM.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

void SLAM::
Print
    (string name)
{
    PRINT_BEGIN(cout);
    
    cout<<"SLAM: " << name << endl;
    mParameters.Print("SLAM.mparameters");
    mPoses.Print("SLAM.mPoses");
    mConstraints.Print("SLAM.mConstraints");

    PRINT_END(cout);
}

// load g2o file
void SLAM::
loadData
    ()
{
    clear();

    cerr << "Load data from file ... " << endl;
    
    if(mParameters.inputFileName == "")
    {
        cerr << "Please specify file name." << endl;
        exit(EXIT_FAILURE);
    }
    
    ifstream is(mParameters.inputFileName.c_str());
    assert(is);
    
    long LINESIZE = 81920;
    
    vector<Constraint *> cstsInAMiniBatch;
    
    while(is)
    {
        char buf[LINESIZE];
        is.getline(buf,LINESIZE);
        istringstream ls(buf);
        string tag;
        ls >> tag;
        
        // g2o format
        if(tag == "VERTEX_SE2")
        {
            int poseID;
            REALTYPE x,y,heading;
            ls >> poseID >> x >> y >> heading;
            mPoses.addPose(poseID, x, y, heading);
        }
        else if(tag=="EDGE_SE2")
        {
            int id1, id2;
            Vector t(3);
            Matrix info(3,3);
            ls >> id1 >> id2 >> t[0] >> t[1] >> t[2];
            
            if(mParameters.identityCov)
            {
                info.Identity();
            }
            else
            {
                ls >> info(0, 0) >> info(0, 1) >> info(0, 2)
                >> info(1, 1) >> info(1, 2) >> info(2, 2);
            }
            
            info(1,0)=info(0,1);
            info(2,0)=info(0,2);
            info(2,1)=info(1,2);
            
            int idx1,idx2;
            idx1 = mPoses.findPoseIdxByID(id1);
            idx2 = mPoses.findPoseIdxByID(id2);
            
            assert(idx1>= 0);
            assert(idx2>= 0);
            assert(idx1 != idx2);
            
            // ignore relative motion constraint
            if ((fabs(id1-id2) == 1))
            {
                continue;
            }
            
            Constraint *cst = mConstraints.addConstraint(id1,id2,idx1,idx2,t,info);
            cstsInAMiniBatch.push_back(cst);
        }
        else if(tag=="MiniBatchStart")
        {
            cstsInAMiniBatch.clear();
        }
        else if(tag=="MiniBatchEnd")
        {
            assert(cstsInAMiniBatch.size() > 0);
            mConstraints.addMiniBatch(cstsInAMiniBatch);
            cstsInAMiniBatch.clear();
        }

    }
    
    cerr << "Done" << endl;
    cout << "Number of Poses: " << mPoses.numPoses << endl;
    cout << "Number of Constraints: " << mConstraints.numCsts << endl;
    cout << "Number of Mini-batches: " << mConstraints.numMiniBatches << endl;
    
    init();
}

void SLAM::
saveGraph
    (string name)
{
    cout << "Saving graph: " << name << endl;
    
    string graphFileNameStr;
    if (mParameters.outputFolderPath == "")
    {
        string strippedFileName = mParameters.strippedInputFilename();
        graphFileNameStr = strippedFileName + "_" + name;
    }
    else
    {
        graphFileNameStr = mParameters.outputFolderPath + "/" + name;
    }
    
    // g2o format
    graphFileNameStr += ".g2o";
    
    const char* graphFileName = graphFileNameStr.c_str();
    
    ofstream os(graphFileName);
    assert(os);
    PRINT_BEGIN(os);

    updatePoses();
    
    // save vertices
    for(int i  = 0; i < mPoses.numPoses; i++)
    {
        os << "VERTEX_SE2 "
           << (int)mPoses.ids.RefNoCheck(i) << " "
           << mPoses.xs.RefNoCheck(i) << " "
           << mPoses.ys.RefNoCheck(i) << " "
           << mPoses.headings.RefNoCheck(i) << endl;
    }
    
    // save edges
    for (vector<Constraint *>::iterator itCstP = mConstraints.constraints.begin(); itCstP != mConstraints.constraints.end(); itCstP++)
    {
        Constraint *cst = *itCstP;
        
        os << "EDGE_SE2 " << cst->aID << " " << cst->bID << " ";
        os << cst->t[0] << " " << cst->t[1] << " " << cst->t[2] << " ";
        os << cst->info(0,0) << " "
           << cst->info(0,1) << " "
           << cst->info(0,2) << " "
           << cst->info(1,1) << " "
           << cst->info(1,2) << " "
           << cst->info(2,2) << endl;
    }

    PRINT_END(os);
    cout << "Done." << endl << endl;
}
