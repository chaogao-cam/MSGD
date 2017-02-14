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


#include <iostream>

#include "SLAM/SLAM.h"


#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

using namespace std;

const char*  message[] =
{
    "",
    " Usage msgd [options]",
    " options ",
    " -in  <str>  input g2o file name",
    " -out <str>  the path to the output folder",
    " -i   <int>  perform <int> iterations. Default: 100",
    " -v          show errors at the end of every iteration",
    " -dr         disable egde randomnization",
    " -ic         set constraint covariance to idendity matrix",
    " -sa         save results for all iterations",
    0
};

bool parseParas(int argc, const char *argv[], SLAM
                &slam)
{
    if(argc==1)
    {
        int i=0;
        while(message[i]!=0)
        {
            cerr << message[i++] << endl;
        }
        return false;
    }
    
    int c=1;
    while(c<argc)
    {
        if(string(argv[c])=="-in")
        {
            slam.mParameters.inputFileName = argv[++c];
        }
        else if(string(argv[c])=="-out")
        {
            slam.mParameters.outputFolderPath = argv[++c];
        }
        else if(string(argv[c])=="-i")
        {
            slam.mParameters.iterations = atoi(argv[++c]);
        }
        else if(string(argv[c])=="-v")
        {
            slam.mParameters.showProgress = true;
        }
        else if(string(argv[c])=="-dr")
        {
            slam.mParameters.enableRandom = false;
        }
        else if(string(argv[c])=="-ic")
        {
            slam.mParameters.identityCov = true;
        }
        else if(string(argv[c])=="-sa")
        {
            slam.mParameters.saveAllIters = true;
        }
        
        c++;
    }
    
    if(slam.mParameters.inputFileName == "")
    {
        cerr << "Please specify the data file name." << endl;
        return  false;
    }
    
    cerr << "*******************************************************************" << endl;
    cerr << " Input File                                      = " << slam.mParameters.inputFileName << endl;
    cerr << " Output Folder                                   = " << slam.mParameters.outputFolderPath << endl;
    cerr << " Iterations                                      = " << slam.mParameters.iterations << endl;
    cerr << " Show Progress                                   = " << ((slam.mParameters.showProgress) ? "Yes" : "No") << endl;
    cerr << " Enable Edge Randomnization                      = " << ((slam.mParameters.enableRandom) ? "Yes" : "No") << endl;
    cerr << " Set Constraint Covariances to Idendity Matrix   = " << ((slam.mParameters.identityCov) ? "Yes" : "No") << endl;
    cerr << " Save Result Graph For Every Iteration           = " << ((slam.mParameters.saveAllIters) ? "Yes" : "No") << endl;
    cerr << "*******************************************************************" << endl;
    
    cerr << endl;
    
    return true;
}

int main
    (int argc, const char *argv[])
{
    SLAM slam;

    if(!parseParas(argc, argv, slam))
    {
        return 0;
    }
    
    slam.loadData();
    slam.optimise();

    return 0;
}
