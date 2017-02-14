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

#include "SLAMParas.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

void SLAMParas::
Print
    (string name) const
{
    PRINT_BEGIN(cout);
    
    cout<<"SLAMParas: " << name << endl;

    cout << "Paras.inputFileName: " << inputFileName <<endl;
    cout << "Paras.saveAllIters: " << (saveAllIters ? "yes" : "no") << endl;
    cout << "Paras.showProgress: " << (showProgress ? "yes" : "no") << endl;
    cout << "Paras.identityCov: " << (identityCov ? "yes" : "no") << endl;
    cout << "Paras.iterations: " << iterations <<endl;
    cout << "computeHEveryIteration: " << (computeHEveryIteration ? "yes" : "no") << endl;
    cout << "Paras.enableRandom: " << (enableRandom ? "yes" : "no") << endl;
    
    PRINT_END(cout);
}

string SLAMParas::stripExtension
(const string s)
{
    unsigned long i;
    for(i=s.length()-1; i>0; i--)
    {
        string k=s.substr(i,1);
        if(k==".")
        {
            break;
        }
    }
    return s.substr(0,i);
}
