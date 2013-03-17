#pragma once

#include "NURBSCurve.h"
#include "NURBSRectangle.h"

typedef std::pair<Vector3,Vector3> PairVector3;
#define glVector3( v ) glVertex3d( v.x(), v.y(), v.z() )
#define glNormal3( v ) glNormal3d( v.x(), v.y(), v.z() )
#define glLine(v1,v2) glVector3(v1);glVector3(v2)

namespace NURBS
{
    class CurveDraw{
    public:
        static void draw( NURBSCurved * nc, QColor curve_color = QColor(0,255,255), bool drawControl = false);
    };

    class SurfaceDraw{
    public:
        static void draw( NURBSRectangled * nc, QColor sheet_color = QColor(0,255,255), bool drawControl = false );
    };
}
