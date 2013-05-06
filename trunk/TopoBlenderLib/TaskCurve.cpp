#include "TaskCurve.h"
#include "AbsoluteOrientation.h"

using namespace Structure;

Curve * TaskCurve::targetCurve()
{
	Node* n = targetNode();
	if(!n || n->type() != CURVE) return NULL;
	return (Curve *)n;
}


void TaskCurve::prepareCurve()
{
	switch(type)
	{
	case GROW:
		prepareGrowCurve();
		break;
	case SHRINK:
		prepareShrinkCurve();
		break;
	case SPLIT:
	case MERGE:
	case MORPH:
		prepareMorphCurve();
		break;
	}
}


void TaskCurve::prepareShrinkCurveOneEdge( Link* l )
{
	Node * n = node();
	Curve* structure_curve = ((Curve*)n);

	Node * base = l->otherNode(n->id);

	Vec4d coordBase = l->getCoord(base->id).front();
	Vec4d coordSelf = l->getCoord(n->id).front();

	Vector3 linkPositionBase = l->position( base->id );

	// Curve folding
	Array1D_Vector3 deltas = structure_curve->foldTo( coordSelf, false );
	deltas = inverseVectors3(deltas);

	// Growing / shrinking instructions
	property["deltas"].setValue( deltas );
	property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );
}

void TaskCurve::prepareShrinkCurve()
{
	Node *n = node(), *tn = targetNode();
	QVector<Link*> edges = filterEdges( n, active->getEdges(n->id) );
	Curve* curve = ((Curve*)n);

	if(edges.size() == 1)
	{
		prepareShrinkCurveOneEdge( edges.front() );
	}

	if(edges.size() == 2)
	{
		// Links and positions (on myself)
		Link * linkA = edges.front();
		Link * linkB = edges.back();
		Vec3d pointA = linkA->position( n->id );
		Vec3d pointB = linkB->position( n->id );

		// Geodesic distance between two link positions on the active graph excluding the running tasks
		QVector< GraphDistance::PathPointPair > path;
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
		GraphDistance gd( active, exclude );
		gd.computeDistances( pointA, DIST_RESOLUTION );
		gd.smoothPathCoordTo(pointB, path);

		// No path = Cut node case
		if( !path.size() )
		{
			prepareShrinkCurveOneEdge( preferredEnd(n, edges, active) );
			return;
		}

		// Use the center of the path as the end point
		GraphDistance::PathPointPair endPointCoord = path[path.size() / 2];
		Vec3d endPoint = endPointCoord.position(active);

		// Separate the path into two for linkA and linkB
		int N = path.size(), hN = N / 2;
		if (N % 2 == 0) {
			path.insert(hN, path[hN]);
			N++;
		}

		QVector< GraphDistance::PathPointPair > pathA, pathB;
		for (int i = 0; i < N/2; i++)
		{
			pathA.push_back( path[N-1-i] );
			pathB.push_back( path[i] );
		}

		property["pathA"].setValue( GraphDistance::positionalPath(active,pathA) );
		property["pathB"].setValue( GraphDistance::positionalPath(active,pathB) );

		// Save edges used
		QVector<Link*> edges;
		edges.push_back( linkA ); edges.push_back( linkB );
		property["edges"].setValue( edges );

		// Encode curve
		property["cpCoords"].setValue( Curve::encodeCurve(curve, linkA->position(n->id), linkB->position(n->id)) );

		// Visualization
		n->property["path"].setValue( GraphDistance::positionalPath(active, pathA) );
		n->property["path2"].setValue( GraphDistance::positionalPath(active, pathB) );
	}
}

void TaskCurve::prepareGrowCurve()
{
	Node * n = node();
	Node * tn = targetNode();
	QVector<Link*> all_tedges = target->getEdges(tn->id);

	Curve * curve = (Curve *)n;
	Curve * tcurve = (Curve *)tn;

	// Do not consider edges with non-ready nodes
	std::vector<Link*> tedges;
	foreach(Link* edge, all_tedges){
		Node * tother = edge->otherNode( tn->id );
		Node * other = active->getNode( tother->property["correspond"].toString() );
		if( other->property.contains("taskIsDone") )
			tedges.push_back(edge);
	}

	if( all_tedges.empty() ) return;

	// Cut nodes grow case
	if( tedges.empty() )
	{
		tedges.push_back( preferredEnd(tn, all_tedges, target) );
	}

	if (tedges.size() == 1)
	{
		Link * tlink = tedges.front();
		Node * tbase = tlink->otherNode(tn->id);

		Vec4d coordBase = tlink->getCoord(tbase->id).front();
		Vec4d coordSelf = tlink->getCoord(tn->id).front();

		QString baseID = tbase->property["correspond"].toString();
		Node* base = active->getNode(baseID);

		Vector3 linkPositionBase = base->position(coordBase);
		int cpIDX = curve->controlPointIndexFromCoord( coordSelf );

		/// Place curve:

		// Make origin the position on me in which I will grow from
		curve->moveBy( -n->position(coordSelf)  );

		Vec3d endDelta = tlink->position(tn->id) - tlink->positionOther(tn->id);
		QString corrBaseNodeID = tlink->otherNode(tn->id)->property["correspond"].toString();
		Vec3d posOther = active->getNode(corrBaseNodeID)->position(tlink->getCoordOther(tn->id).front());
		if(n->property["isCutGroup"].toBool()) endDelta = Vec3d(0);

		// Move curve to start point computed by looking at target geometry
		curve->moveBy( posOther + endDelta );

		// Curve folding
		Array1D_Vector3 deltas = curve->foldTo( coordSelf, true );

		// Growing instructions
		property["deltas"].setValue( deltas );
		property["orgCtrlPoints"].setValue( curve->curve.mCtrlPoint );
	}

	if (tedges.size() == 2)
	{
		// Links, nodes and positions on the TARGET
		Link *tlinkA = tedges.front();
		Link *tlinkB = tedges.back();
		Node *totherA = tlinkA->otherNode( tn->id );
		Node *totherB = tlinkB->otherNode( tn->id );
		Vec4d othercoordA = tlinkA->getCoord(totherA->id).front();
		Vec4d othercoordB = tlinkB->getCoord(totherB->id).front();

		// Corresponding stuff on ACTIVE
		Node *otherA = active->getNode( totherA->property["correspond"].toString() );
		Node *otherB = active->getNode( totherB->property["correspond"].toString() );
		Vec3d pointA = otherA->position(othercoordA);
		Vec3d pointB = otherB->position(othercoordB);

		// Geodesic distance between two link positions on the active graph excluding the running tasks
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
		GraphDistance gd( active, exclude );
		gd.computeDistances( pointA, DIST_RESOLUTION );
		QVector< GraphDistance::PathPointPair > path;
		gd.smoothPathCoordTo(pointB, path);
		path = weldPath( path );

		// Use the center of the path as the start point
		if(path.size() < 1) return;
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

		property["pathA"].setValue( GraphDistance::positionalPath(active,pathA) );
		property["pathB"].setValue( GraphDistance::positionalPath(active,pathB) );

		QVector<Link*> edges;
		edges.push_back( active->getEdge( tlinkA->property["correspond"].toString() ) );
		edges.push_back( active->getEdge( tlinkB->property["correspond"].toString() ) );
		property["edges"].setValue( edges );

		// Encode curve
		property["cpCoords"].setValue( Structure::Curve::encodeCurve(tcurve, tlinkA->position(tn->id), tlinkB->position(tn->id)) );

		// Initial position of curve node
		Vec4d midCoord(0.5);
		curve->foldTo(midCoord, true);
		curve->curve.translate(startPoint - curve->position(midCoord));

		// Visualization
		n->property["path"].setValue( GraphDistance::positionalPath(active, pathA) );
		n->property["path2"].setValue( GraphDistance::positionalPath(active, pathB) );
	}
}


void TaskCurve::prepareMorphCurve()
{
	Node * n = node(), *tn = targetNode();

	//// Parameters needed for morphing
	Vector3 sourceA = n->position(Vec4d(0)), sourceB = n->position(Vec4d(1));
	Vector3 targetA = tn->position(Vec4d(0)), targetB = tn->position(Vec4d(1));

	// Two ends
	property["sourceA"].setValue( sourceA ); property["sourceB"].setValue( sourceB );
	property["targetA"].setValue( targetA ); property["targetB"].setValue( targetB );

	// Encoding
	property["cpCoords"].setValue( Structure::Curve::encodeCurve((Curve*)n, sourceA, sourceB) );
	property["cpCoordsT"].setValue( Structure::Curve::encodeCurve((Curve*)tn, targetA, targetB) );

	// Check crossing
	if ( isCrossing() ) 
		prepareCrossingMorphCurve();
	else			  
		prepareMorphEdges();
}

void TaskCurve::prepareCrossingMorphCurve()
{
	Node * n = node();

	QVector<Link*> edges = filterEdges(n, active->getEdges(n->id));

	if( edges.size() == 2 )
	{
		// Start and end for both links
		Link * linkA = edges.front();
		Vec3d startA = linkA->position(n->id);
		NodeCoord edA = futureLinkCoord(linkA);
		Vec3d endA = active->position(edA.first,edA.second);

		Link * linkB = edges.back();
		Vec3d startB = linkB->position(n->id);
		NodeCoord edB = futureLinkCoord(linkB);
		Vec3d endB = active->position(edB.first,edB.second);

		// Geodesic distances on the active graph excluding the running tasks
		QVector< GraphDistance::PathPointPair > pathA, pathB;
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
		GraphDistance gdA( active, exclude ), gdB( active, exclude );

		gdA.computeDistances( endA, DIST_RESOLUTION );  gdA.smoothPathCoordTo( startA, pathA );
		gdB.computeDistances( endB, DIST_RESOLUTION );  gdB.smoothPathCoordTo( startB, pathB );

		property["pathA"].setValue( GraphDistance::positionalPath(active,pathA) );
		property["pathB"].setValue( GraphDistance::positionalPath(active,pathB) );
	}

	property["edges"].setValue( edges );
}



void TaskCurve::executeCurve(double t)
{
	switch(type)
	{
	case GROW:
	case SHRINK:
		executeCrossingCurve(t);
		break;
	case MORPH:
		{
			if( property["isCrossing"].toBool() )
				executeCrossingCurve(t);
			else
				executeMorphCurve(t);
		}
		break;
	}
}


void TaskCurve::foldCurve( double t )
{
	Curve* structure_curve = ((Curve*)node());

	Array1D_Vector3 cpts = property["orgCtrlPoints"].value<Array1D_Vector3>();
	Array1D_Vector3 deltas = property["deltas"].value<Array1D_Vector3>();

	if(cpts.size() != deltas.size()) return;

	// Grow curve
	for(int u = 0; u < structure_curve->curve.mNumCtrlPoints; u++)
		structure_curve->curve.mCtrlPoint[u] = cpts[u] + (deltas[u] * t);
}

void TaskCurve::executeCrossingCurve( double t )
{
	Node *n = node(), *tn = targetNode();
	QVector<Link*> edges = property["edges"].value< QVector<Link*> >();

	// Regular shrink
	if ( property.contains("deltas") )
	{
		foldCurve(t);
	}
	else if(property.contains("pathA") && property.contains("pathB"))
	{
		Array1D_Vector3 pathA = property["pathA"].value< Array1D_Vector3 >();
		Array1D_Vector3 pathB = property["pathB"].value< Array1D_Vector3 >();

		int nA = pathA.size(), nB = pathB.size();

		if(nA == 0 || nB == 0)  return;

		int idxA = t * (pathA.size() - 1);
		int idxB = t * (pathB.size() - 1);

		// Move to next step
		Vector3 pointA = pathA[idxA];
		Vector3 pointB = pathB[idxB];

		// Decode
		SheetEncoding cpCoords = property["cpCoords"].value<SheetEncoding>();
		SheetEncoding cpCoordsT = property["cpCoordsT"].value<SheetEncoding>();

		double dt = t;
		if(type == SHRINK) dt = 1 - t;

		// Delta
		Structure::Link *slinkA = edges.front(), *slinkB = edges.back();
		Vec3d sDeltaA = slinkA->property["delta"].value<Vec3d>();
		Vec3d sDeltaB = slinkB->property["delta"].value<Vec3d>();

		Structure::Link* tlinkA = target->getEdge(slinkA->property["correspond"].toString());
		Structure::Link* tlinkB = target->getEdge(slinkB->property["correspond"].toString());
		Vec3d tDeltaA = slinkA->property["delta"].value<Vec3d>();
		Vec3d tDeltaB = slinkB->property["delta"].value<Vec3d>();

		Vec3d deltaA = AlphaBlend(dt, sDeltaA, tDeltaA);
		Vec3d deltaB = AlphaBlend(dt, sDeltaB, tDeltaB);

		// Delta checking:
		if( !isConsistant(slinkA, tlinkA) ) deltaA *= -1;
		if( isConsistant(slinkB, tlinkB) ) deltaB *= -1;

		// Visualize
		active->vs.addVector(pointA, deltaA);
		active->vs.addVector(pointB, deltaB);

		Array1D_Vector3 newPnts = Curve::decodeCurve(property["cpCoords"].value<CurveEncoding>(), pointA + deltaA, pointB + deltaB);
		Array1D_Vector3 newPntsT = Curve::decodeCurve(property["cpCoordsT"].value<CurveEncoding>(), pointA + deltaA, pointB + deltaB);

		if(!newPntsT.size()) newPntsT = newPnts;

		Array1D_Vector3 blendedPnts;

		for(int i = 0; i < (int)newPnts.size(); i++)
		{
			blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );
		}

		n->setControlPoints( blendedPnts );
	}

	// When task is done
	if ( t >= 1.0 )
	{
		QVector<Link*> edges = property["edges"].value< QVector<Link*> >();

		if(type == GROW)
		{

		}

		if(type == SHRINK)
		{
			n->property["isReady"] = false;
		}

		// Update to target edges
		if( edges.size() == 2 )
		{
			Link *linkA = edges.front();
			Link *linkB = edges.back();

			Link *tlinkA = target->getEdge(linkA->property["correspond"].toString());
			Link *tlinkB = target->getEdge(linkB->property["correspond"].toString());

			Node * futureOtherA = active->getNode( tlinkA->otherNode(tn->id)->property["correspond"].toString() );
			Vec4d futureCoordA = tlinkA->getCoordOther(tn->id).front();
			linkA->replace( linkA->otherNode(n->id)->id, futureOtherA, Array1D_Vec4d(1, futureCoordA) );
			
			Node * futureOtherB = active->getNode( tlinkB->otherNode(tn->id)->property["correspond"].toString() );
			Vec4d futureCoordB = tlinkB->getCoordOther(tn->id).front();
			linkB->replace( linkB->otherNode(n->id)->id, futureOtherB, Array1D_Vec4d(1, futureCoordB) );
		}
	}
}

void TaskCurve::executeMorphCurve( double t )
{
	Node *n = node(), *tn = targetNode();

	// Blend controlling "line segment"
	Vector3 pointA = AlphaBlend(t, property["sourceA"].value<Vector3>(), property["targetA"].value<Vector3>());
	Vector3 pointB = AlphaBlend(t, property["sourceB"].value<Vector3>(), property["targetB"].value<Vector3>());

	// Blend geometry of skeleton
	SheetEncoding cpCoords = property["cpCoords"].value<SheetEncoding>();
	SheetEncoding cpCoordsT = property["cpCoordsT"].value<SheetEncoding>();
	Array1D_Vector3 newPnts = Curve::decodeCurve(property["cpCoords"].value<CurveEncoding>(), pointA, pointB);
	Array1D_Vector3 newPntsT = Curve::decodeCurve(property["cpCoordsT"].value<CurveEncoding>(), pointA, pointB);

	Array1D_Vector3 blendedPnts;
	for(int i = 0; i < (int)newPnts.size(); i++) blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );
	n->setControlPoints( blendedPnts );
}

