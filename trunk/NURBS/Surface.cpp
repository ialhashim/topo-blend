// Geometric Tools, LLC
// Copyright (c) 1998-2012
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
//
// File Version: 5.0.1 (2010/10/01)


#include "Surface.h"

namespace NURBS
{
//----------------------------------------------------------------------------
template <typename Real>
Surface<Real>::Surface ()
{
}
//----------------------------------------------------------------------------
// Explicit instantiation.
//----------------------------------------------------------------------------
//template
//class Surface<float>;

template
class Surface<double>;
//----------------------------------------------------------------------------
}
