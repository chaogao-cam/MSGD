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

#include "Constraint.h"

#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

void Constraint::
Print
(string name) const
{
    PRINT_BEGIN(cout);
    
    cout<<"Constraint: " << name << endl;
    cout<<"Constraint.aIdx: " << aIdx << " aID: " << aID << endl;
    cout<<"Constraint.bIdx: " << bIdx << " bID: " << bID << endl;
    cout<<"Constraint.chi2Error: " << chi2Error << endl;

//    t.Print("Constraint.t");
//    info.Print("Constraint.info");
//    rotA.Print("Constraint.rotA");
//    transA.Print("Constraint.transA");
    
    PRINT_END(cout);
}
