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
	else if( isCutting() )
	{
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
		gd.smoothPathCoordTo( pointB, path);

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
	tcurve.moveBy( -tcurve.controlPoints()[cpIDX] + base->position( coordBase ) );

	// Curve folding
	Array1D_Vector3 deltas = tcurve.foldTo( coordSelf, true );

	// Growing instructions
	property["deltas"].setValue( deltas );
	property["orgCtrlPoints"].setValue( tcurve.curve.mCtrlPoint );

	// Force blended delta
	Structure::Link * slink = active->getEdge(tlink->property["correspond"].toInt());
	slink->property["blendedDelta"].setValue( tlink->delta() );

	property["isGrowSingleEdge"] = true;
}

void TaskCurve::prepareGrowCurve()
{
	Node *n = node(), *tn = targetNode();
	Curve * curve = (Curve *)n;
	Curve * tcurve = (Curve *)tn;

	// Find edges to be used in growing
	QVector<Link*> tedges;
	QVector<Link*> edges = filteredFromTargetEdges();
	foreach(Link* edge, edges) tedges.push_back(target->getEdge( edge->property["correspond"].toInt() ));
 
	// Save the edges used
	property["edges"].setValue( edges );

	if (edges.size() == 1)
	{
		prepareGrowCurveOneEdge( tedges.front() );
		return;
	}

	if (edges.size() && (isCutting(true) || target->isCutNode(tn->id)))
	{
		prepareGrowCurveOneEdge( tedges.front() );
		return;
	}

	if (edges.size() > 1)
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

		// Very short paths
		if(path.size() < 4){
			prepareGrowCurveOneEdge( tedges.front() );
			return;
		}

		// Use the center of the path as the start point
		GraphDistance::PathPointPair startPointCoord = path[path.size() / 2];
		Vector3d startPoint = startPointCoord.position( active );

		// Separate the path into two for linkA and linkB
		int N = path.size(), hN = N / 2;
		if (N % 2 == 0) path.insert(hN, path[hN]);

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

void TaskCurve::encodeCurve( const Vector4d& coordinateA, const Vector4d& coordinateB )
{
	Node * n = node(), *tn = targetNode();

	//// Parameters needed for morphing
	Vector3 sourceA = n->position(coordinateA), sourceB = n->position(coordinateB);
	Vector3 targetA = tn->position(coordinateA), targetB = tn->position(coordinateB);
	
	// Case: looped curves
	{
		Vector4d mid( 0.5, 0.5, 0.5, 0.5 );
		Vector3 sourceMid = n->position(mid), targetMid = tn->position(mid);

		// Source
		{
			double distBA = (sourceB - sourceA).norm(), distAM = (sourceA - sourceMid).norm(), distBM = (sourceB - sourceMid).norm();
			if( distBA < qMax(distAM, distBM) )
				distAM < distBM ? sourceA = sourceMid : sourceB = sourceMid;
		}

		// Target
		{
			double distBA = (targetB - targetA).norm(), distAM = (targetA - targetMid).norm(), distBM = (targetB - targetMid).norm();
			if( distBA < qMax(distAM, distBM) )
				distAM < distBM ? targetA = targetMid : targetB = targetMid;
		}
	}

	// Two ends
	property["sourceA"].setValue( sourceA ); property["sourceB"].setValue( sourceB );
	property["targetA"].setValue( targetA ); property["targetB"].setValue( targetB );

	// Encoding
	property["cpCoords"].setValue( Structure::Curve::encodeCurve((Curve*)n, sourceA, sourceB) );
	property["cpCoordsT"].setValue( Structure::Curve::encodeCurve((Curve*)tn, targetA, targetB) );
}

void TaskCurve::prepareMorphCurve()
{
	encodeCurve( Vector4d(0,0,0,0), Vector4d(1,1,1,1) );

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
	
	// Case: filter out edges that will change end to non-grown nodes
	if( true )
	{
		QVector<Link*> keep;
		foreach(Link* l, edges){
			NodeCoord futureNodeCord = futureLinkCoord(l);
			if(!ungrownNode(futureNodeCord.first))
				keep.push_back(l);
		}
		if(keep.size()) edges = keep;
	}

	if( edges.isEmpty() )
	{
		edges.push_back( active->getEdges(n->id).front() );
	}

	if (edges.size() == 1)
	{
		// Start and end
		Link * link = edges.front();
		Vector3d start = link->positionOther(n->id);
		NodeCoord futureNodeCord = futureLinkCoord(link);
		Vector3d end = active->position(futureNodeCord.first,futureNodeCord.second);

		// Geodesic distances on the active graph excluding the running tasks
		QVector< GraphDistance::PathPointPair > path;
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();

		// Exclude un-grown cutting
		{
			foreach(Node * n, active->nodes)
			{
				if( futureNodeCord.first != n->id && ungrownNode(n->id) )
				{
					bool isCuttingNode = n->property["task"].value<Task*>()->isCutting(true);

					if( isCuttingNode ) 
						continue;
					else
						exclude.push_back( n->id );
				}
			}
		}

		GraphDistance gd( active, exclude );
		gd.computeDistances( end, DIST_RESOLUTION );  
		gd.smoothPathCoordTo( start, path );

		// Check
		if(path.size() < 2) { path.clear(); path.push_back(GraphDistance::PathPointPair( PathPoint(futureNodeCord.first, futureNodeCord.second))); }

		property["path"].setValue( GraphDistance::positionalPath(active, path) );

		// Save links paths
		path.back() = GraphDistance::PathPointPair( PathPoint(futureNodeCord.first, futureNodeCord.second)  );
		link->setProperty("path", path);

		property["isSingleCrossing"] = true;
	}

	if( edges.size() == 2 )
	{
		// Start and end for both links
		Link * linkA = edges.front();
		Link * linkB = edges.back();
		
		NodeCoord futureNodeCordA = futureLinkCoord(linkA);
		NodeCoord futureNodeCordB = futureLinkCoord(linkB);

		Vector3d startA = linkA->positionOther(n->id);
		Vector3d endA = active->position(futureNodeCordA.first,futureNodeCordA.second);

		Vector3d startB = linkB->positionOther(n->id);
		Vector3d endB = active->position(futureNodeCordB.first,futureNodeCordB.second);

		// Geodesic distances on the active graph excluding the running tasks
		QVector< GraphDistance::PathPointPair > pathA, pathB;
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
		GraphDistance gdA( active, exclude ), gdB( active, exclude );

		gdA.computeDistances( endA, DIST_RESOLUTION );  
		gdA.smoothPathCoordTo( startA, pathA );

		gdB.computeDistances( endB, DIST_RESOLUTION );  
		gdB.smoothPathCoordTo( startB, pathB );

		// Checks
		if(pathA.size() < 2) { pathA.clear(); pathA.push_back(GraphDistance::PathPointPair( PathPoint(futureNodeCordA.first, futureNodeCordA.second))); }
		if(pathB.size() < 2) { pathB.clear(); pathB.push_back(GraphDistance::PathPointPair( PathPoint(futureNodeCordB.first, futureNodeCordB.second))); }

		property["pathA"].setValue( GraphDistance::positionalPath(active, pathA) );
		property["pathB"].setValue( GraphDistance::positionalPath(active, pathB) );

		// Make sure we end correctly
		pathA.back() = GraphDistance::PathPointPair( PathPoint(futureNodeCordA.first, futureNodeCordA.second)  );
		pathB.back() = GraphDistance::PathPointPair( PathPoint(futureNodeCordB.first, futureNodeCordB.second)  );

		// Save links paths
		linkA->setProperty("path", pathA);
		linkB->setProperty("path", pathB);

		// Encode with these two links
		encodeCurve(linkA->getCoord(n->id).front(), linkB->getCoord(n->id).front());
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
			if ( property.contains("deltas") )	foldCurve(t);
			else executeCrossingCurve(t);
		}
		break;
	case MORPH:
		{
			if( property["isCrossing"].toBool())
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
	QVector<Link*> edges = property["edges"].value< QVector<Link*> >();

	// Only valid edges
	QVector<Link*> keep;
	foreach(Link* l, edges) if(l->hasNode(n->id)) keep.push_back(l);
	edges = keep;

	// Something went wrong..
	if(edges.isEmpty())	edges = active->getEdges(n->id);

    Link * l = edges.front();

	Vector3 posOnMe = l->position(n->id);
	Vector3 posOnBase = l->positionOther(n->id);

	// Get delta 
	Vector3 delta = l->property["blendedDelta"].value<Vector3d>();

	if(this->type == Task::GROW) 
	{
		Link * tl = target->getEdge( l->property["correspond"].toInt() );
		delta = tl->delta();
	}

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
}

void TaskCurve::executeMorphCurve( double t )
{
    Node *n = node();

	// Blend controlling "line segment"
	Vector3 pointA = AlphaBlend(t, property["sourceA"].value<Vector3>(), property["targetA"].value<Vector3>());
	Vector3 pointB = AlphaBlend(t, property["sourceB"].value<Vector3>(), property["targetB"].value<Vector3>());

	// Blend geometry of skeleton
	CurveEncoding cpCoords = property["cpCoords"].value<CurveEncoding>();
	CurveEncoding cpCoordsT = property["cpCoordsT"].value<CurveEncoding>();
	Array1D_Vector3 newPnts = Curve::decodeCurve(property["cpCoords"].value<CurveEncoding>(), pointA, pointB);
	Array1D_Vector3 newPntsT = Curve::decodeCurve(property["cpCoordsT"].value<CurveEncoding>(), pointA, pointB);

	Array1D_Vector3 blendedPnts;
	for(int i = 0; i < (int)newPnts.size(); i++) 
		blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );
	
	n->setControlPoints( blendedPnts );
}
