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

	Vector4d coordBase = l->getCoord(base->id).front();
	Vector4d coordSelf = l->getCoord(n->id).front();
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
    Node *n = node();
	QVector<Link*> edges = filterEdges( n, active->getEdges(n->id) );

	if(!edges.size()) return; // Something went wrong

	Curve* curve = ((Curve*)n);

	// Save edges used
	property["edges"].setValue( edges );

	if(edges.size() == 1)
	{
		prepareShrinkCurveOneEdge( edges.front() );
	}

	//Cut node case
	else if( isCutting())
	{
		property["isCutNode"] = true;
		prepareShrinkCurveOneEdge( preferredEnd(n, edges, active) );
	}

	else if(edges.size() == 2)
	{
		// Links and positions (on myself)
		Link * linkA = edges.front();
		Link * linkB = edges.back();
		Vector3d pointA = linkA->position( n->id );
		Vector3d pointB = linkB->position( n->id );

		// Geodesic distance between two link positions on the active graph excluding the running tasks and ungrown tasks
		QVector< GraphDistance::PathPointPair > path;
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
		foreach(Node* node, active->nodes){ if (ungrownNode(node->id)) exclude.push_back(node->id);}
		GraphDistance gd( active, exclude );
		gd.computeDistances( pointA, DIST_RESOLUTION );
		gd.smoothPathCoordTo(pointB, path);

		if(path.size() == 0) return;

		// Use the center of the path as the end point
		GraphDistance::PathPointPair endPointCoord = path[path.size() / 2];
		Vector3d endPoint = endPointCoord.position(active);

		// Separate the path into two for linkA and linkB
		// Need old number of positions to shrink to null
		int N = path.size(), hN = N / 2;
		if (N % 2 == 0)	path.insert(hN, path[hN]);
		N = path.size();
		QVector< GraphDistance::PathPointPair > pathA, pathB;
		for (int i = 0; i <= N/2; i++)
		{
			pathA.push_back( path[N-1-i] );
			pathB.push_back( path[i] );
		}

		property["pathA"].setValue( GraphDistance::positionalPath(active,pathA) );
		property["pathB"].setValue( GraphDistance::positionalPath(active,pathB) );

		// Encode curve
		property["cpCoords"].setValue( Curve::encodeCurve(curve, linkA->position(n->id), linkB->position(n->id)) );

		linkA->setProperty("path", pathA);
		linkB->setProperty("path", pathB);

		// Visualization
		n->property["path"].setValue( GraphDistance::positionalPath(active, pathA) );
		n->property["path2"].setValue( GraphDistance::positionalPath(active, pathB) );
	}
}

void TaskCurve::prepareGrowCurveOneEdge( Structure::Link * tlink )
{
    Node *tn = targetNode();
	Structure::Curve tcurve (*((Structure::Curve *)tn));

	// Get base
	Node * tbase = tlink->otherNode(tn->id);
	QString baseID = tbase->property["correspond"].toString();
	Node* base = active->getNode(baseID);

	// Get coordinates on me and my base
	Vector4d coordBase = tlink->getCoord(tbase->id).front();
	Vector4d coordSelf = tlink->getCoord(tn->id).front();

	/// Place curve:
	int cpIDX = tcurve.controlPointIndexFromCoord( coordSelf );

	// Make origin the position on me in which I will grow from
	tcurve.moveBy( -tcurve.controlPoints()[cpIDX] + base->position(coordBase) );

	// Curve folding
	Array1D_Vector3 deltas = tcurve.foldTo( coordSelf, true );

	// Growing instructions
	property["deltas"].setValue( deltas );
	property["orgCtrlPoints"].setValue( tcurve.curve.mCtrlPoint );
}

void TaskCurve::prepareGrowCurve()
{
	Node *n = node(), *tn = targetNode();
	QVector<Link*> all_tedges = target->getEdges(tn->id);

	Curve * curve = (Curve *)n;
	Curve * tcurve = (Curve *)tn;

	// Isolated branch growth: do not consider edges with non-ready nodes
	QVector<Link*> tedges, edges;
	foreach(Link* edge, all_tedges)
	{
		Node * tother = edge->otherNode( tn->id );
		Node * other = active->getNode( tother->property["correspond"].toString() );

		// Skip not grown others
		if( all_tedges.size() > 1 && ungrownNode(other->id) )	continue;

		tedges.push_back(edge);		
	}

	// Find corresponding edges
	foreach(Link * edge, tedges)
	{
		Link * slink = active->getEdge( edge->property["correspond"].toInt() );
		if(slink) edges.push_back( slink );
	}

	// Save edges used
	property["edges"].setValue( edges );

	if (tedges.size() == 1)
	{
		prepareGrowCurveOneEdge( tedges.front() );
		return;
	}

	if (tedges.size() && isCutting())
	{
		property["isCutNode"] = true;

		prepareGrowCurveOneEdge( tedges.front() );
		return;
	}

	if (tedges.size() > 1)
	{
		// Make sure the edges are reasonable for curve encoding
		QMap< double, QPair<int,int> > edgePairs;
		for(int i = 0; i < tedges.size(); i++){
			Link *tlinkA = tedges[i];
			for(int j = i + 1; j < tedges.size(); j++){
				Link *tlinkB = tedges[j];
				double dist = (tlinkA->position(tn->id) - tlinkB->position(tn->id)).norm();
				edgePairs[dist] = qMakePair(i,j);
			}
		}

		// Prefer most distant edges
		double largestDist = edgePairs.keys().last();
		if( largestDist < 1e-6 ){
			prepareGrowCurveOneEdge( tedges.front() );
			return;
		}

		// Links, nodes and positions on the TARGET
		Link *tlinkA = tedges[ edgePairs[largestDist].first ];
		Link *tlinkB = tedges[ edgePairs[largestDist].second ];
		Node *totherA = tlinkA->otherNode( tn->id );
		Node *totherB = tlinkB->otherNode( tn->id );
		Vector4d othercoordA = tlinkA->getCoord(totherA->id).front();
		Vector4d othercoordB = tlinkB->getCoord(totherB->id).front();

		// Corresponding stuff on ACTIVE
		Link *linkA = active->getEdge( tlinkA->property["correspond"].toInt() );
		Link *linkB = active->getEdge( tlinkB->property["correspond"].toInt() );

		if(!linkA || !linkB) return;

		Node *otherA = active->getNode( totherA->property["correspond"].toString() );
		Node *otherB = active->getNode( totherB->property["correspond"].toString() );
		Vector3d pointA = otherA->position(othercoordA);
		Vector3d pointB = otherB->position(othercoordB);

		// Geodesic distance between two link positions on the active graph excluding the running tasks
		QVector<QString> excludeNodes = active->property["activeTasks"].value< QVector<QString> >();
		
		GraphDistance gd( active, excludeNodes );
		gd.computeDistances( pointA, DIST_RESOLUTION );
		QVector< GraphDistance::PathPointPair > path;
        NodeCoord rpoint( otherB->id, othercoordB );
        gd.smoothPathCoordTo( rpoint, path );

		if(path.size() == 0) return;

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

		property["pathA"].setValue( GraphDistance::positionalPath(active,pathA) );
		property["pathB"].setValue( GraphDistance::positionalPath(active,pathB) );

		QVector<Link*> edges;
		edges.push_back( active->getEdge( tlinkA->property["correspond"].toInt() ) );
		edges.push_back( active->getEdge( tlinkB->property["correspond"].toInt() ) );
		property["edges"].setValue( edges );

		// Encode curve
		property["cpCoords"].setValue( Structure::Curve::encodeCurve(tcurve, tlinkA->position(tn->id), tlinkB->position(tn->id)) );

		// Initial position of curve node
		Vector4d midCoord(0.5,0.5,0.5,0.5);
		curve->foldTo(midCoord, true);
		curve->curve.translate(startPoint - curve->position(midCoord));

		linkA->setProperty("path", pathA);
		linkB->setProperty("path", pathB);

		// Visualization
		n->property["path"].setValue( GraphDistance::positionalPath(active, pathA) );
		n->property["path2"].setValue( GraphDistance::positionalPath(active, pathB) );

		return;
	}
}

void TaskCurve::prepareMorphCurve()
{
	Node * n = node(), *tn = targetNode();

	//// Parameters needed for morphing
	Vector3 sourceA = n->position(Vector4d(0,0,0,0)), sourceB = n->position(Vector4d(1,1,1,1));
	Vector3 targetA = tn->position(Vector4d(0,0,0,0)), targetB = tn->position(Vector4d(1,1,1,1));

	// Two ends
	property["sourceA"].setValue( sourceA ); property["sourceB"].setValue( sourceB );
	property["targetA"].setValue( targetA ); property["targetB"].setValue( targetB );

	// Encoding
	property["cpCoords"].setValue( Structure::Curve::encodeCurve((Curve*)n, sourceA, sourceB) );
	property["cpCoordsT"].setValue( Structure::Curve::encodeCurve((Curve*)tn, targetA, targetB) );

	// Check crossing
	if ( (isReady = true) && isCrossing() ) 
		prepareCrossingMorphCurve();
	else			  
		prepareMorphEdges();
}

void TaskCurve::prepareCrossingMorphCurve()
{
	Node * n = node();

	QVector<Link*> edges = filterEdges(n, active->getEdges(n->id));

	if (edges.size() == 1)
	{
		// Start and end for both links
		Link * link = edges.front();
		Vector3d start = link->position(n->id);
		NodeCoord futureNodeCord = futureLinkCoord(link);
		Vector3d end = active->position(futureNodeCord.first,futureNodeCord.second);

		// Geodesic distances on the active graph excluding the running tasks
		QVector< GraphDistance::PathPointPair > path;
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
		GraphDistance gd( active, exclude );
		gd.computeDistances( end, DIST_RESOLUTION );  
		gd.smoothPathCoordTo( start, path );

		// Check
		if(path.isEmpty()) path.push_back(GraphDistance::PathPointPair( PathPoint(futureNodeCord.first, futureNodeCord.second)));

		property["path"].setValue( GraphDistance::positionalPath(active, path) );

		// Save links paths
		path.back() = GraphDistance::PathPointPair( PathPoint(futureNodeCord.first, futureNodeCord.second)  );
		link->setProperty("path", path);
	}

	if( edges.size() == 2 )
	{
		// Start and end for both links
		Link * linkA = edges.front();
		Vector3d startA = linkA->position(n->id);
		NodeCoord futureNodeCordA = futureLinkCoord(linkA);
		Vector3d endA = active->position(futureNodeCordA.first,futureNodeCordA.second);

		Link * linkB = edges.back();
		Vector3d startB = linkB->position(n->id);
		NodeCoord futureNodeCordB = futureLinkCoord(linkB);
		Vector3d endB = active->position(futureNodeCordB.first,futureNodeCordB.second);

		// Geodesic distances on the active graph excluding the running tasks
		QVector< GraphDistance::PathPointPair > pathA, pathB;
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
		GraphDistance gdA( active, exclude ), gdB( active, exclude );

		gdA.computeDistances( endA, DIST_RESOLUTION );  gdA.smoothPathCoordTo( startA, pathA );
		gdB.computeDistances( endB, DIST_RESOLUTION );  gdB.smoothPathCoordTo( startB, pathB );

		// Checks
		if(pathA.isEmpty()) pathA.push_back(GraphDistance::PathPointPair( PathPoint(futureNodeCordA.first, futureNodeCordA.second)));
		if(pathB.isEmpty()) pathB.push_back(GraphDistance::PathPointPair( PathPoint(futureNodeCordB.first, futureNodeCordB.second)));

		property["pathA"].setValue( GraphDistance::positionalPath(active, pathA) );
		property["pathB"].setValue( GraphDistance::positionalPath(active, pathB) );

		// Save links paths
		pathA.back() = GraphDistance::PathPointPair( PathPoint(futureNodeCordA.first, futureNodeCordA.second)  );
		pathB.back() = GraphDistance::PathPointPair( PathPoint(futureNodeCordB.first, futureNodeCordB.second)  );
		
		linkA->setProperty("path", pathA);
		linkB->setProperty("path", pathB);
	}

	property["edges"].setValue( edges );
}

void TaskCurve::executeCurve(double t)
{
	switch(type)
	{
	case GROW:
	case SHRINK:
		{
			// DEBUG:
			//if(type==GROW)active->spheres.addSphere(this->node()->bbox().center(), 0.03, QColor(255,0,0,100));

			if ( property.contains("deltas") )	foldCurve(t);
			else executeCrossingCurve(t);
		}
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
	Node * n = node();
	Curve* structure_curve = ((Curve*)n);

	// Grow curve
	Array1D_Vector3 cpts = property["orgCtrlPoints"].value<Array1D_Vector3>();
	Array1D_Vector3 deltas = property["deltas"].value<Array1D_Vector3>();

	if(cpts.size() != deltas.size()) return;
	for(int u = 0; u < structure_curve->curve.mNumCtrlPoints; u++)
		structure_curve->curve.mCtrlPoint[u] = cpts[u] + (deltas[u] * t);

	// Placement
    Link * l = property["edges"].value< QVector<Link*> >().front();

	Vector3 posOnMe = l->position(n->id);
	Vector3 posOnBase = l->positionOther(n->id);

	// Get delta 
	Vector3 delta = l->property["blendedDelta"].value<Vector3d>();

	// Delta to me
	if(l->n1->id == n->id) delta *= -1;

	structure_curve->moveBy( posOnBase + delta - posOnMe );
}

void TaskCurve::executeCrossingCurve( double t )
{
    Node *n = node();
	QVector<Link*> edges = property["edges"].value< QVector<Link*> >();

	if (property.contains("path"))
	{
		// Blend the geometry
		executeMorphCurve(t);

		// Move it to the correct position
		Array1D_Vector3 path = property["path"].value< Array1D_Vector3 >();
		int idx = t * (path.size() - 1);
		Vector3 point = path[idx];

		Link* link = edges.front();
		Vector3 oldPos = link->position(n->id);

		// Blend Deltas, directions are the same as source
        Structure::Link *slink = edges.front();
		Vector3d sDelta = slink->property["delta"].value<Vector3d>();
		if (type == Task::GROW) sDelta = Vector3d(0,0,0);

		Structure::Link* tlink = target->getEdge(slink->property["correspond"].toInt());
		Vector3d tDelta = tlink->property["delta"].value<Vector3d>();
		if (type == Task::SHRINK) tDelta = Vector3d(0,0,0);

		Vector3d delta = AlphaBlend(t, sDelta, tDelta);

		// Deltas to myself
		if (slink->n1->id == n->id) delta *= -1;

		Vector3 newPos = point + delta;

		// Place
		n->moveBy(newPos - oldPos);
	}

	// Walk along using two edges
	if(property.contains("pathA") && property.contains("pathB"))
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

		// Blend Deltas, directions are the same as source
		Structure::Link *slinkA = edges.front(), *slinkB = edges.back();
		Vector3d sDeltaA = slinkA->property["delta"].value<Vector3d>();
		Vector3d sDeltaB = slinkB->property["delta"].value<Vector3d>();
		if (type == Task::GROW) {sDeltaA = Vector3d(0,0,0); sDeltaB = Vector3d(0,0,0);}

		Structure::Link* tlinkA = target->getEdge(slinkA->property["correspond"].toInt());
		Structure::Link* tlinkB = target->getEdge(slinkB->property["correspond"].toInt());
		Vector3d tDeltaA = tlinkA->property["delta"].value<Vector3d>();
		Vector3d tDeltaB = tlinkB->property["delta"].value<Vector3d>();
		if (type == Task::SHRINK) {tDeltaA = Vector3d(0,0,0); tDeltaB = Vector3d(0,0,0);}

		Vector3d deltaA = AlphaBlend(t, sDeltaA, tDeltaA);
		Vector3d deltaB = AlphaBlend(t, sDeltaB, tDeltaB);

		// Deltas to myself
		if (slinkA->n1->id == n->id) deltaA *= -1;
		if (slinkB->n1->id == n->id) deltaB *= -1;

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
			
		}
	}
}

void TaskCurve::executeMorphCurve( double t )
{
    Node *n = node();

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

