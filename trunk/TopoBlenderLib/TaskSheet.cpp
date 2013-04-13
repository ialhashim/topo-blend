#include "TaskSheet.h"
#include "AbsoluteOrientation.h"

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
        executeGrowShrinkSheet(t);
        break;
    case MORPH:
        executeMorphSheet(t);
        break;
    }
}

void TaskSheet::prepareSheetOneEdge( Structure::Link * l )
{
    Structure::Node * n = node();
    Structure::Node * base = l->otherNode(n->id);
    Structure::Sheet* structure_sheet = ((Structure::Sheet*)n);

    // Placement:
    structure_sheet->moveBy( l->position( base->id ) - l->position( n->id ) );

    // Sheet folding:
    Array2D_Vector3 deltas = structure_sheet->foldTo( l->getCoord(n->id), (this->type == GROW) );
    if (this->type != GROW) deltas = inverseVectors3(deltas);

    // Growing / shrinking instructions
    property["deltas"].setValue( deltas );
    property["orgCtrlPoints"].setValue( structure_sheet->surface.mCtrlPoint );
}

void TaskSheet::prepareSheetTwoEdges( Structure::Link * linkA, Structure::Link * linkB )
{
    Structure::Node * n = node();

    // Corresponding stuff on ACTIVE
    Vec3d pointA = linkA->positionOther(n->id);
    Vec3d pointB = linkB->positionOther(n->id);

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
    Vec3d startPoint = startPointCoord.position( active );

    // Separate the path into two for linkA and linkB
    int N = path.size(), hN = N / 2;
    if (N %2 == 0) path.insert(hN, path[hN]);

    QVector<GraphDistance::PathPointPair> pathA, pathB;
    for (int i = 0; i < hN; i++)
    {
        pathA.push_back(path[hN+1+i]);
        pathB.push_back(path[hN-1-i]);
    }

    // Add smooth ending on both paths
    //Vec3d endDeltaA = tn->position(tlinkA->getCoord(tn->id).front()) - totherA->position(othercoordA);
    //Vec3d endDeltaB = tn->position(tlinkB->getCoord(tn->id).front()) - totherB->position(othercoordB);
    //Node * auxA = new Structure::Curve(NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, pointA + endDeltaA ) ), "auxA_" + n->id);
    //Node * auxB = new Structure::Curve(NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, pointB + endDeltaB ) ), "auxB_" + n->id);
    //active->aux_nodes.push_back( auxA );
    //active->aux_nodes.push_back( auxB );
    //pathA = smoothEnd(auxA, Vec4d(0), pathA);
    //pathB = smoothEnd(auxB, Vec4d(0), pathB);

    // Record path
    property["pathA"].setValue( pathA );
    property["pathB"].setValue( pathB );

    // Encode curve
    RMF rmf( positionalPath(pathA, 1) );
    property["rmf"].setValue( rmf );
    if(!rmf.count()) return;

    Vector3 X = rmf.U.back().r, Y = rmf.U.back().s, Z = rmf.U.back().t;

    // Encode sheet on a line segment
    SheetEncoding cpCoords = encodeSheetAsCurve((Structure::Sheet*)n, linkA->position(n->id), linkB->position(n->id), X,Y,Z);
    property["cpCoords"].setValue( cpCoords );

    // DEBUG frames
    node()->property["rmf"].setValue( rmf );
    node()->property["rmf2"].setValue( RMF ( positionalPath(pathB, 3) ) );

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
    property["rotation"].setValue( rotation );
    property["sframe"].setValue( sframe );
    property["tframe"].setValue( tframe );

    property["cpCoords"].setValue( encodeSheet(sheet, sframe.center, sframe.r,sframe.s,sframe.t) );
    property["cpCoordsT"].setValue( encodeSheet(tsheet, tframe.center, tframe.r,tframe.s,tframe.t) );

    // Visualization
    std::vector<RMF::Frame> frames = smoothRotateFrame(sframe, rotation, 100);
    property["frames"].setValue( frames );
    n->property["frames"].setValue( frames );
    n->property["frame"].setValue( sframe );
    tn->property["frame"].setValue( tframe );
}


void TaskSheet::executeGrowShrinkSheet( double t )
{
    Structure::Sheet* structure_sheet = ((Structure::Sheet*)node());

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
        if(this->type == SHRINK) dt = 1 - t;

		double decodeT = qMin(1.0, dt * 2.0);

        int idxA = dt * (pathA.size() - 1);
        int idxB = dt * (pathB.size() - 1);

        // Move to next step
        Vector3 pointA = pathA[idxA].position(active);
        Vector3 pointB = pathB[idxB].position(active);

        RMF rmf = property["rmf"].value<RMF>();
        Vector3 X = rmf.frameAt(dt).r, Y = rmf.frameAt(dt).s, Z = rmf.frameAt(dt).t;
        Array1D_Vector3 decoded = decodeCurve(property["cpCoords"].value<CurveEncoding>(), pointA, pointB, X,Y,Z, decodeT);
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

            // Delete all edges
            //foreach(Structure::Link *link, edges)
            //{
            //    active->removeEdge(link->n1, link->n2);
            //}
        }

        if (type == GROW)
        {
            if(tedges.size()) copyTargetEdge(tedges.front());
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
    Eigen::Quaterniond rotation = property["rotation"].value<Eigen::Quaterniond>(),
        eye = Eigen::Quaterniond::Identity();

    // Source sheet
    Eigen::Vector3d R = V2E(sframe.r), S = V2E(sframe.s), T = V2E(sframe.t);
    R = eye.slerp(t, rotation) * R;
    S = eye.slerp(t, rotation) * S;
    T = eye.slerp(t, rotation) * T;
    RMF::Frame curFrame (E2V(R),E2V(S),E2V(T));

    curFrame.center = tframe.center = AlphaBlend(t, sframe.center, tframe.center);

    Array1D_Vector3 newPnts = decodeSheet( cpCoords, curFrame.center, curFrame.r, curFrame.s, curFrame.t );
    Array1D_Vector3 newPntsT = decodeSheet( cpCoordsT, tframe.center, tframe.r, tframe.s, tframe.t );

    Array1D_Vector3 blendedPnts;

    for(int i = 0; i < (int)newPnts.size(); i++)
    {
        blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );
    }

    sheet->setControlPoints( blendedPnts );
}

SheetEncoding TaskSheet::encodeSheetAsCurve( Structure::Sheet * sheet, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z )
{
    Array1D_Vector3 controlPoints = sheet->controlPoints();
    return encodeCurve(controlPoints,start,end,X,Y,Z);
}

Array1D_Vector3 TaskSheet::decodeSheetFromCurve( double t, SheetEncoding cpCoords, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z )
{
    return decodeCurve(cpCoords,start,end,X,Y,Z,t);
}

Structure::Sheet * TaskSheet::targetSheet()
{
    Structure::Node* n = targetNode();
    if(!n || n->type() != Structure::SHEET) return NULL;
    return (Structure::Sheet *)n;
}
