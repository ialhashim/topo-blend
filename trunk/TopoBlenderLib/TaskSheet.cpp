#include "TaskSheet.h"
#include "AbsoluteOrientation.h"

#include "StructureCurve.h"

using namespace Structure;

void TaskSheet::prepareSheet()
{
    switch(type)
    {
    case GROW:
    case SHRINK:
        prepareGrowShrinkSheet();
        break;
    case SPLIT:
    case MERGE:
    case MORPH:
        prepareMorphSheet();
        break;
    }
}

void TaskSheet::executeSheet(double t)
{
    switch(type)
    {
    case GROW:
    case SHRINK:
        executeCrossingSheet(t);
        break;
    case MORPH:
		{
			if( property["isCrossing"].toBool() )
				executeCrossingSheet(t);
			else
				executeMorphSheet(t);
		}
        break;
    }
}

void TaskSheet::prepareSheetOneEdge( Structure::Link * l )
{
    Structure::Node * n = node(), *tn = targetNode();
    Structure::Node * base = l->otherNode(n->id);
    Structure::Sheet* structure_sheet = ((Structure::Sheet*)n);

    // Placement:
    structure_sheet->moveBy( l->position( base->id ) - l->position( n->id ) );

    // Sheet folding:
	Structure::Sheet targetCopy (*((Structure::Sheet*)tn));
    Array2D_Vector3 deltas = targetCopy.foldTo( l->getCoord(n->id), (this->type == GROW) );
    if (this->type != GROW) deltas = inverseVectors3(deltas);

    // Growing / shrinking instructions
    property["deltas"].setValue( deltas );
    property["orgCtrlPoints"].setValue( structure_sheet->surface.mCtrlPoint );
}

void TaskSheet::prepareSheetTwoEdges( Structure::Link * linkA, Structure::Link * linkB )
{
    Structure::Node * n = node();
	Structure::Node * tn = targetNode();

	Structure::Link * tlinkA = target->getEdge( linkA->property["correspond"].toInt() );
	Structure::Link * tlinkB = target->getEdge( linkB->property["correspond"].toInt() );

    Vector3d pointA = linkA->positionOther(n->id);
    Vector3d pointB = linkB->positionOther(n->id);

    // Geodesic distance between two link positions on the active graph excluding the running tasks
    QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
    GraphDistance gd( active, exclude );
    gd.computeDistances( pointA, DIST_RESOLUTION );
    QVector< GraphDistance::PathPointPair > path;
    gd.smoothPathCoordTo(pointB, path);
    path = weldPath( path );

    // Otherwise, self expand / contract
    if(path.size() < 1)
    {
        QVector<Link*> twoEdges(2);
        twoEdges[0] = linkA; twoEdges[1] = linkB;
        prepareSheetOneEdge(preferredEnd(n, twoEdges, active));
        return;
    }

    // Use the center of the path as the start point
    GraphDistance::PathPointPair startPointCoord = path[path.size() / 2];
    Vector3d startPoint = startPointCoord.position( active );

    // Separate the path into two for linkA and linkB
    int N = path.size(), hN = N / 2;
    if (N %2 == 0) path.insert(hN, path[hN]);

    QVector<GraphDistance::PathPointPair> pathA, pathB;
    for (int i = 0; i < hN; i++)
    {
        pathA.push_back(path[hN+1+i]);
        pathB.push_back(path[hN-1-i]);
    }

    // Record path
    property["pathA"].setValue( pathA );
    property["pathB"].setValue( pathB );

    // Encode sheet on a line segment
	RMF rmf( GraphDistance::positionalPath(active, pathA, 1) );
	property["rmf"].setValue( rmf );
	if(!rmf.count()) return;

	Vector3 X = rmf.U.back().r, Y = rmf.U.back().s, Z = rmf.U.back().t;

    SheetEncoding cpCoords = encodeSheetAsCurve((Structure::Sheet*)tn, tlinkA->position(tn->id), tlinkB->position(tn->id));
	property["cpCoords"].setValue( cpCoords );

    // DEBUG
    node()->property["rmf"].setValue( rmf );
    node()->property["rmf2"].setValue( RMF ( GraphDistance::positionalPath(active, pathB, 3) ) );

    if(this->type == GROW)
    {
        // Initial position and geometry
        n->setControlPoints( Array1D_Vector3(cpCoords.size(), startPoint) );
    }
}

void TaskSheet::prepareGrowShrinkSheet()
{
    Structure::Node * n = node();
    QVector<Structure::Link*> edges = filterEdges(n, active->getEdges(n->id));

	property["edges"].setValue( edges );

    if (edges.size() == 1)
    {
        prepareSheetOneEdge( edges.front() );
    }

    if (edges.size() == 2)
    {
        prepareSheetTwoEdges( edges.front(), edges.back() );
    }
}

void TaskSheet::prepareMorphSheet()
{
    // Morph
    Structure::Node * n = node();
    Structure::Node * tn = targetNode();
    Structure::Sheet * sheet = (Structure::Sheet *)n;
    Structure::Sheet * tsheet = (Structure::Sheet *)tn;

    // Get source and target frames
    RMF::Frame sframe = sheetFrame( sheet );
    RMF::Frame tframe = sheetFrame( tsheet );

    // Compute rotations between source and target sheet frames
    Eigen::Quaterniond rotation, eye = Eigen::Quaterniond::Identity();
    AbsoluteOrientation::compute(sframe.r,sframe.s,sframe.t,
                                 tframe.r,tframe.s,tframe.t, rotation);

    // Parameters needed for morphing
	QVector<double> rot; 
	rot.push_back(rotation.w());
	rot.push_back(rotation.x());
	rot.push_back(rotation.y());
	rot.push_back(rotation.z());

    property["rotation"].setValue( rot );
    property["sframe"].setValue( sframe );
    property["tframe"].setValue( tframe );

    property["cpCoords"].setValue( Structure::Sheet::encodeSheet(sheet, sframe.center, sframe.r,sframe.s,sframe.t) );
    property["cpCoordsT"].setValue( Structure::Sheet::encodeSheet(tsheet, tframe.center, tframe.r,tframe.s,tframe.t) );

    // Visualization
    std::vector<RMF::Frame> frames = smoothRotateFrame(sframe, rotation, 100);
    property["frames"].setValue( frames );
    n->property["frames"].setValue( frames );
    n->property["frame"].setValue( sframe );
    tn->property["frame"].setValue( tframe );

	prepareMorphEdges();
}


void TaskSheet::executeCrossingSheet( double t )
{
    Structure::Sheet* structure_sheet = ((Structure::Sheet*)node());
	QVector<Link*> edges = property["edges"].value< QVector<Link*> >();

    /// Single edge case
    if ( property.contains("deltas") )
    {
        Array2D_Vector3 cpts = property["orgCtrlPoints"].value<Array2D_Vector3>();
        Array2D_Vector3 deltas = property["deltas"].value<Array2D_Vector3>();

        // Grow sheet
        for(int u = 0; u < structure_sheet->surface.mNumUCtrlPoints; u++)
            for(int v = 0; v < structure_sheet->surface.mNumVCtrlPoints; v++)
                structure_sheet->surface.mCtrlPoint[u][v] = cpts[u][v] + (deltas[u][v] * t);
    }

    /// Two edges case
    if( property.contains("pathA") && property.contains("pathB") && property.contains("cpCoords") )
    {
        QVector< GraphDistance::PathPointPair > pathA = property["pathA"].value< QVector< GraphDistance::PathPointPair > >();
        QVector< GraphDistance::PathPointPair > pathB = property["pathB"].value< QVector< GraphDistance::PathPointPair > >();
        if(pathA.size() == 0 || pathB.size() == 0)	return;

        double dt = t;
        if(type == SHRINK) dt = 1 - t;

		double decodeT = qMin(1.0, dt * 2.0);

        int idxA = dt * (pathA.size() - 1);
        int idxB = dt * (pathB.size() - 1);

        // Move to next step
        Vector3 pointA = pathA[idxA].position(active);
        Vector3 pointB = pathB[idxB].position(active);

		Structure::Link *linkA = edges.front(), *linkB = edges.back();
		if (type == GROW){
			linkA = target->getEdge(linkA->property["correspond"].toInt());
			linkB = target->getEdge(linkB->property["correspond"].toInt());
		}

		Vector3d deltaA = linkA->property["delta"].value<Vector3d>() * dt;
		Vector3d deltaB = linkB->property["delta"].value<Vector3d>() * dt;
		
		// Visualize
		active->vs.addVector(pointA,deltaA);
		active->vs.addVector(pointB,deltaB);
		
        Array1D_Vector3 decoded = Curve::decodeCurve(property["cpCoords"].value<CurveEncoding>(), pointA + deltaA, pointB + deltaB, decodeT);
        structure_sheet->setControlPoints( decoded );
    }

    // When the task is done
    if ( t == 1 )
    {
        Structure::Node * n = node();
        Structure::Node * tn = targetNode();
        QVector<Structure::Link*> edges = active->getEdges(n->id);
        QVector<Structure::Link*> tedges = target->getEdges(tn->id);

        if (type == SHRINK)
        {
            n->property["isReady"] = false;
        }

        if (type == GROW)
        {

        }
    }
}

void TaskSheet::executeMorphSheet( double t )
{
    Structure::Node * n = node();
    Structure::Sheet * sheet = (Structure::Sheet *)n;

    // Decode
    SheetEncoding cpCoords = property["cpCoords"].value<SheetEncoding>();
    SheetEncoding cpCoordsT = property["cpCoordsT"].value<SheetEncoding>();

    RMF::Frame sframe = property["sframe"].value<RMF::Frame>();
    RMF::Frame tframe = property["tframe"].value<RMF::Frame>();
	QVector<double> rot = property["rotation"].value< QVector<double> >();
    Eigen::Quaterniond rotation(rot[0],rot[1],rot[2],rot[3]),
        eye = Eigen::Quaterniond::Identity();

    // Source sheet
    Eigen::Vector3d R = (sframe.r), S = (sframe.s), T = (sframe.t);
    R = eye.slerp(t, rotation) * R;
    S = eye.slerp(t, rotation) * S;
    T = eye.slerp(t, rotation) * T;
    RMF::Frame curFrame ((R),(S),(T));

    curFrame.center = tframe.center = AlphaBlend(t, sframe.center, tframe.center);

    Array1D_Vector3 newPnts = Sheet::decodeSheet( cpCoords, curFrame.center, curFrame.r, curFrame.s, curFrame.t );
    Array1D_Vector3 newPntsT = Sheet::decodeSheet( cpCoordsT, tframe.center, tframe.r, tframe.s, tframe.t );

    Array1D_Vector3 blendedPnts;

    for(int i = 0; i < (int)newPnts.size(); i++)
    {
        blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );
    }

    sheet->setControlPoints( blendedPnts );
}

SheetEncoding TaskSheet::encodeSheetAsCurve( Structure::Sheet * sheet, Vector3 start, Vector3 end)
{
    Array1D_Vector3 controlPoints = sheet->controlPoints();
    return Structure::Curve::encodeCurve(controlPoints,start,end);
}

Array1D_Vector3 TaskSheet::decodeSheetFromCurve( double t, SheetEncoding cpCoords, Vector3 start, Vector3 end)
{
    return Structure::Curve::decodeCurve(cpCoords,start,end,t);
}

Structure::Sheet * TaskSheet::targetSheet()
{
    Structure::Node* n = targetNode();
    if(!n || n->type() != Structure::SHEET) return NULL;
    return (Structure::Sheet *)n;
}
