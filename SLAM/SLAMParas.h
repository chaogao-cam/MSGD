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

#ifndef __MSGD__SLAMParas__
#define __MSGD__SLAMParas__

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
    
    class SLAMParas
    {
    protected:
        
    public:
        
        int iterations; // total iterations to run
        bool saveAllIters; // save result graph for every iteration
        bool showProgress; // show errors at the end of each iteration
        bool identityCov; // set constraint covariance to identity matrix
        bool computeHEveryIteration; // compute preconditioner at every iteration
        bool enableRandom; // enable radomnization
        
        string inputFileName;
        string outputFolderPath;

        inline SLAMParas
        ()
        {
            setDedault();
        }
        
        inline virtual ~SLAMParas
        ()
        {
            Release();
        }
        
        inline virtual void Release
        ()
        {
        }
        
        // default settings
        inline void setDedault()
        {
            iterations = 100;
            computeHEveryIteration = false;
            enableRandom = true;
            identityCov = false;
            showProgress = false;
            saveAllIters = false;
            inputFileName = "";
            outputFolderPath = "";
        }
        
        void Print
        (string name = "") const;
        
        string strippedInputFilename()
        {
            return stripExtension(inputFileName);
        }
        
    private:
        string stripExtension
        (const string s);
    };
    
#ifdef USE_MATHLIB_NAMESPACE
}
#endif


#endif /* defined(__MSGD__SLAMParas__) */
