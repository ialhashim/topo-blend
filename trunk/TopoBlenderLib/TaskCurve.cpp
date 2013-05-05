#include "TaskCurve.h"
#include "AbsoluteOrientation.h"

using namespace Structure;

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

void TaskCurve::executeCurve(double t)
{
	switch(type)
	{
	case GROW:
	case SHRINK:
		executeGrowShrinkCurve(t);
		break;
	case MORPH:
		executeMorphCurve(t);
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
	Node * n = node();
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

		// Smooth transition from start point
		pathA = smoothStart( node(), linkA->getCoord(n->id).front(), pathA );
		pathB = smoothStart( node(), linkB->getCoord(n->id).front(), pathB );

		property["pathA"].setValue( pathA );
		property["pathB"].setValue( pathB );

		// Encode curve
		property["cpCoords"].setValue( Curve::encodeCurve(curve, linkA->position(n->id), linkB->position(n->id)) );

		// Visualization
		n->property["path"].setValue( positionalPath(pathA) );
		n->property["path2"].setValue( positionalPath(pathB) );
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

		// Add smooth ending on both paths
		Vec3d endDeltaA = tn->position(tlinkA->getCoord(tn->id).front()) - totherA->position(othercoordA);
		Vec3d endDeltaB = tn->position(tlinkB->getCoord(tn->id).front()) - totherB->position(othercoordB);
		Node * auxA = new Curve(NURBS::NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, pointA + endDeltaA ) ), "auxA_" + n->id);
		Node * auxB = new Curve(NURBS::NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, pointB + endDeltaB ) ), "auxB_" + n->id);
		active->aux_nodes.push_back( auxA );
		active->aux_nodes.push_back( auxB );
		pathA = smoothEnd(auxA, Vec4d(0), pathA);
		pathB = smoothEnd(auxB, Vec4d(0), pathB);

		property["pathA"].setValue( pathA );
		property["pathB"].setValue( pathB );

		// geometry ends


		// Encode curve
		property["cpCoords"].setValue( Structure::Curve::encodeCurve(tcurve, tlinkA->position(tn->id), tlinkB->position(tn->id)) );

		// Initial position of curve node
		Vec4d midCoord(0.5);
		curve->foldTo(midCoord, true);
		curve->curve.translate(startPoint - curve->position(midCoord));

		// Visualization
		n->property["path"].setValue( positionalPath(pathA) );
		n->property["path2"].setValue( positionalPath(pathB) );
	}
}

void TaskCurve::prepareMorphCurve()
{
	Node * n = node();
	Node * tn = targetNode();

	Curve * curve = (Curve *)n;
	Curve * tcurve = (Curve *)tn;

	// 0) Filter edges (remove edges with null since these nodes will grow in future)
	QVector<Link*> edges = filterEdges(n, active->getEdges(n->id));

	// Make sure local link coordinates are consistent from source to target
	foreach(Link * l, edges)
	{
		Vec4d scoord = l->getCoord(n->id).front();
		Vec4d tcoord = target->getEdge(l->property["correspond"].toString())->getCoord(tn->id).front();

		bool isWithinThreshold = (abs(scoord[0]-tcoord[0]) < 0.5);

		if(isSameHalf(scoord,tcoord) || isWithinThreshold ) continue;
		l->invertCoords(n->id);
	}

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		Link * link = edges.front();
		QString otherNode = link->otherNode(n->id)->id;

		NodeCoord fnc = futureOtherNodeCoord(link);
		NodeCoord ed = futureLinkCoord(link);

		Vec4d coordA = link->getCoord(n->id).front();
		Vec4d coordB = Vec4d( (coordA[0] > 0.5) ? 0 : 1 );

		//visualizeStructureGraph(active, "activeSingleEdge" + n->id);

		// Compute path for the edge
		Vec3d startPoint = n->position( coordA );
		Vec3d endPoint = active->position(ed.first, ed.second);

		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();

		QVector< GraphDistance::PathPointPair > path;
		GraphDistance gd(active, exclude);
		gd.computeDistances( endPoint, DIST_RESOLUTION );
		gd.smoothPathCoordTo( startPoint, path );

		// Smooth transition from start point
		if( !isPathOnSingleNode( path ) )
		{
			path = smoothStart( node(), link->getCoord(n->id).front(), path );
		}
		else
		{
			// Add a "static" point
			path.clear();
			Node * aux = addAuxNode(link->position(n->id), active);
			path.push_back( GraphDistance::PathPointPair( PathPoint (aux->id, Vec4d(0)) ) );
		}

		// Smooth transition to end point
		//path = smoothEnd( prepareEnd(n, link), Vec4d(0), path );

		// Replace coordinates and such
		link->replace( otherNode, active->getNode(fnc.first), std::vector<Vec4d>(1,fnc.second) );

		path = weldPath( path );

		property["path"].setValue( path );

		// Encode source curve
		Vec3d startA = startPoint;
		Vec3d startB = n->position( coordB );
		property["cpCoords"].setValue( Structure::Curve::encodeCurve((Curve*)n, startA, startB) );
		property["endPointDelta"].setValue( startB - startA );

		// Encode target curve
		Vec3d tstartA = tn->position( coordA );
		Vec3d tstartB = tn->position( coordB );
		property["cpCoordsT"].setValue( Structure::Curve::encodeCurve((Curve*)tn, tstartA, tstartB) );
		property["endPointDeltaT"].setValue( tstartB - tstartA );

		// DEBUG:
		node()->property["path"].setValue( positionalPath(path) );
	}

	// 2) TWO edges
	if(edges.size() == 2)
	{
		// Start and end for both links
		Link * linkA = edges.front();
		Vec3d startA = linkA->positionOther(n->id);
		NodeCoord edA = futureOtherNodeCoord(linkA);
		Vec3d endA = active->position(edA.first,edA.second);

		Link * linkB = edges.back();
		Vec3d startB = linkB->positionOther(n->id);
		NodeCoord edB = futureOtherNodeCoord(linkB);
		Vec3d endB = active->position(edB.first,edB.second);

		// Geodesic distances on the active graph excluding the running tasks
		QVector< GraphDistance::PathPointPair > pathA, pathB;
		QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
		GraphDistance gdA( active, exclude ), gdB( active, exclude );

		gdA.computeDistances( endA, DIST_RESOLUTION );	gdA.smoothPathCoordTo( startA, pathA );
		gdB.computeDistances( endB, DIST_RESOLUTION );	gdB.smoothPathCoordTo( startB, pathB );

		// Smooth transition from start point, unless the path only goes through a single node
		{
			if( isPathOnSingleNode(pathA) )
			{
				// Add a "static" point
				pathA.clear();
				Node * aux = addAuxNode(linkA->position(n->id), active);
				pathA.push_back( GraphDistance::PathPointPair( PathPoint (aux->id, Vec4d(0)) ) );
			}
			//else
			//    pathA = smoothStart(node(), linkA->getCoord(n->id).front(), pathA);

			if( isPathOnSingleNode(pathB) )
			{
				// Add a "static" point
				pathB.clear();
				Node * aux = addAuxNode(linkB->position(n->id), active);
				pathB.push_back( GraphDistance::PathPointPair( PathPoint (aux->id, Vec4d(0)) ) );
			}
			//else
			//    pathB = smoothStart(node(), linkB->getCoord(n->id).front(), pathB);
		}

		// Replace destination
		foreach(Link *link, edges){
			QString otherNode = link->otherNode(n->id)->id;
			NodeCoord fnc = futureOtherNodeCoord(link);
			if(!fnc.first.contains("null"))
				link->replace( otherNode, active->getNode(fnc.first), std::vector<Vec4d>(1,fnc.second) );
			else
			{
				// When connected to growing cut node, use edge with cut node instead
				Link * cutLink = active->getEdge( n->id, fnc.first );

				if(cutLink)
				{
					if(linkA->hasNode(otherNode)) linkA = cutLink;
					if(linkB->hasNode(otherNode)) linkB = cutLink;
				}
			}
		}

		// Remove redundancy along paths
		pathA = this->weldPath( pathA );
		pathB = this->weldPath( pathB );

		property["pathA"].setValue( pathA );
		property["pathB"].setValue( pathB );

		// Corresponding links
		Link * tlinkA = target->getEdge(linkA->property["correspond"].toString());
		Link * tlinkB = target->getEdge(linkB->property["correspond"].toString());
		Vec3d tstartA = tlinkA->position(tn->id);
		Vec3d tstartB = tlinkB->position(tn->id);

		// Check for reversed coordinates
		bool isFlip = false;
		bool isFlipA = false, isFlipB = false;
		if( !isSameHalf(linkA->getCoord(n->id).front(), tlinkA->getCoord(tn->id).front()) )	isFlipA = true;
		if( !isSameHalf(linkB->getCoord(n->id).front(), tlinkB->getCoord(tn->id).front()) )	isFlipB = true;
		isFlip = isFlipA && isFlipB;

		// Parameters needed for morphing
		property["isFlip"].setValue( isFlip );

		Vector3 sourceA = n->position(Vec4d(0)), sourceB = n->position(Vec4d(1));
		Vector3 targetA = tn->position(Vec4d(0)), targetB = tn->position(Vec4d(1));

		property["cpCoords"].setValue( Structure::Curve::encodeCurve((Curve*)n, sourceA, sourceB) );
		property["cpCoordsT"].setValue( Structure::Curve::encodeCurve((Curve*)tn, targetA, targetB, isFlip) );

		// Absolute end points
		property["sourceA"].setValue( sourceA );
		property["sourceB"].setValue( sourceB );

		property["targetA"].setValue( targetA );
		property["targetB"].setValue( targetB );

		// Two ends
		property["linkA"].setValue( linkA );
		property["linkB"].setValue( linkB );

		// Visualization
		n->property["path"].setValue( positionalPath(pathA) );
		n->property["path2"].setValue( positionalPath(pathB) );
	}
}

void TaskCurve::executeGrowShrinkCurve( double t )
{
	Node *n = node();

	// Regular shrink
	if (property.contains("deltas"))
		foldCurve(t);
	else if (property.contains("pathA") && property.contains("pathB"))
	{
		QVector< GraphDistance::PathPointPair > pathA = property["pathA"].value< QVector< GraphDistance::PathPointPair > >();
		QVector< GraphDistance::PathPointPair > pathB = property["pathB"].value< QVector< GraphDistance::PathPointPair > >();

		int nA = pathA.size(), nB = pathB.size();

		if(nA == 0 || nB == 0)  return;

		int idxA = t * (pathA.size() - 1);
		int idxB = t * (pathB.size() - 1);

		// Move to next step
		Vector3 pointA = pathA[idxA].position(active);
		Vector3 pointB = pathB[idxB].position(active);

		// Decode
		SheetEncoding cpCoords = property["cpCoords"].value<SheetEncoding>();
		SheetEncoding cpCoordsT = property["cpCoordsT"].value<SheetEncoding>();
		bool isFlip = property["isFlip"].toBool();

		Array1D_Vector3 newPnts = Curve::decodeCurve(property["cpCoords"].value<CurveEncoding>(), pointA, pointB);
		Array1D_Vector3 newPntsT = Curve::decodeCurve(property["cpCoordsT"].value<CurveEncoding>(), pointA, pointB);

		if(!newPntsT.size()) newPntsT = newPnts;

		Array1D_Vector3 blendedPnts;

		for(int i = 0; i < (int)newPnts.size(); i++)
		{
			blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );
		}

		n->setControlPoints( blendedPnts );
	}

	// When task is done
	if (t == 1)
	{
		QVector<Link*> edges = active->getEdges(n->id);

		if(type == GROW)
		{

		}

		if(type == SHRINK)
		{
			n->property["isReady"] = false;

			// Delete all edges
			foreach(Link *link, edges){
				if(link->property.contains("changingEnd"))
					continue;
				//active->removeEdge(link->n1, link->n2);
			}

			//Curve* structure_curve = ((Curve*)node());
			//structure_curve->foldTo(Vec4d(0),true);
		}
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

void TaskCurve::executeMorphCurve( double t )
{
	Node * n = node();
	Node * tn = targetNode();

	Curve* structure_curve = ((Curve*)n);

	// 1) SINGLE edge
	if( property.contains("path") )
	{
		// Parameters
		QVector< GraphDistance::PathPointPair > path = property["path"].value< QVector< GraphDistance::PathPointPair > >();
		if(!path.size()) return;

		CurveEncoding cpCoords = property["cpCoords"].value<CurveEncoding>();
		CurveEncoding cpCoordsT = property["cpCoordsT"].value<CurveEncoding>();
		Vec3d endPointDelta = property["endPointDelta"].value<Vec3d>();
		Vec3d endPointDeltaT = property["endPointDeltaT"].value<Vec3d>();

		int idx = t * (path.size() - 1);

		Vec3d newHandlePos = path[idx].position(active); // Other end = newHandlePos + endPointDelta

		// Decode and blend
		Array1D_Vector3 newPnts = Curve::decodeCurve( cpCoords, newHandlePos, newHandlePos + endPointDelta );
		Array1D_Vector3 newPntsT = Curve::decodeCurve( cpCoordsT, newHandlePos, newHandlePos + endPointDeltaT );
		Array1D_Vector3 blendedPnts;

		for(int i = 0; i < (int)newPnts.size(); i++)
			blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );

		n->setControlPoints( blendedPnts );
	}

	// 2) TWO edges
	if( property.contains("pathA") && property.contains("pathB") && property.contains("cpCoords") )
	{
		QVector< GraphDistance::PathPointPair > pathA = property["pathA"].value< QVector< GraphDistance::PathPointPair > >();
		QVector< GraphDistance::PathPointPair > pathB = property["pathB"].value< QVector< GraphDistance::PathPointPair > >();

		int nA = pathA.size(), nB = pathB.size();
		if(nA == 0 || nB == 0)	return;

		if (node()->id == "RightDownBar")
		{
			int a = 0;
		}

		Structure::Link * linkA = property["linkA"].value<Structure::Link*>();
		Structure::Link * linkB = property["linkB"].value<Structure::Link*>();
		if(!linkA || !linkB) return;

		// Local link coordinates
		int idxA = t * (pathA.size() - 1);
		int idxB = t * (pathB.size() - 1);

		GraphDistance::PathPointPair curA = pathA[idxA];
		GraphDistance::PathPointPair curB = pathB[idxB];

		linkA->replace( linkA->otherNode(n->id)->id, active->getNode(curA.a.first), std::vector<Vec4d>(1,curA.a.second) );
		linkB->replace( linkB->otherNode(n->id)->id, active->getNode(curB.a.first), std::vector<Vec4d>(1,curB.a.second) );

		// Shape and size of curve
		Vector3 pointA = AlphaBlend(t, property["sourceA"].value<Vector3>(), property["targetA"].value<Vector3>());
		Vector3 pointB = AlphaBlend(t, property["sourceB"].value<Vector3>(), property["targetB"].value<Vector3>());

		// Decode
		SheetEncoding cpCoords = property["cpCoords"].value<SheetEncoding>();
		SheetEncoding cpCoordsT = property["cpCoordsT"].value<SheetEncoding>();

		Array1D_Vector3 newPnts = Curve::decodeCurve(property["cpCoords"].value<CurveEncoding>(), pointA, pointB);
		Array1D_Vector3 newPntsT = Curve::decodeCurve(property["cpCoordsT"].value<CurveEncoding>(), pointA, pointB);

		if(!newPntsT.size()) newPntsT = newPnts;

		Array1D_Vector3 blendedPnts;

		for(int i = 0; i < (int)newPnts.size(); i++)
		{
			blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );
		}

		structure_curve->setControlPoints( blendedPnts );
	}

	// When this task is done
	if (t == 1.0)
	{
		// There are two edges in the active should be killed
		if (type == Task::SHRINK)
		{
			QVector<Link*> edges = active->getEdges(n->id);
			if (edges.size() == 2)
			{
				Link *linkA = edges.front();
				Link *linkB = edges.back();

				//active->removeEdge(linkA->n1, linkA->n2);
				//active->removeEdge(linkB->n1, linkB->n2);
			}
		}

		// There are no edge in active but two in the target
		//if (type == Task::GROW)
		//{
		//	copyTargetEdge(tedges.front());
		//	copyTargetEdge(tedges.back());
		//}
	}
}

Curve * TaskCurve::targetCurve()
{
	Node* n = targetNode();
	if(!n || n->type() != CURVE) return NULL;
	return (Curve *)n;
}
