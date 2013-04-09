#include "DynamicGraph.h"
#include "Scheduler.h"
#include "Task.h"
#include <QGraphicsSceneMouseEvent>

#include "Synthesizer.h"
#include "weld.h"
#include "LineSegment.h"

#include "AbsoluteOrientation.h"

#include "ExportDynamicGraph.h"

using namespace NURBS;
using namespace Structure;

typedef std::vector< std::pair<double,double> > VectorPairDouble;

Q_DECLARE_METATYPE( Vector3 )
Q_DECLARE_METATYPE( Vec4d )
Q_DECLARE_METATYPE( VectorPairDouble )
Q_DECLARE_METATYPE( GraphDistance::PathPointPair )
Q_DECLARE_METATYPE( QVector< GraphDistance::PathPointPair > )
Q_DECLARE_METATYPE( RMF )
Q_DECLARE_METATYPE( RMF::Frame )
Q_DECLARE_METATYPE( std::vector<RMF::Frame> )
Q_DECLARE_METATYPE( CurveEncoding )
Q_DECLARE_METATYPE( Eigen::Quaterniond )

static inline double rad_to_deg(const double& _angle){ return 180.0*(_angle/M_PI); }

Task::Task( Structure::Graph * activeGraph, Structure::Graph * targetGraph, TaskType taskType, int ID )
{
	// Task properties
	this->active = activeGraph;
	this->target = targetGraph;

	this->start = 0;
	this->length = 80;

	this->type = taskType;
	this->taskID = ID;
	this->mycolor = TaskColors[taskType];
	this->currentTime = start;
	this->isReady = false;
	this->isDone = false;

	// Visual properties
	isResizing = false;
	width = length;
	height = 17;
	setFlags( ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges );
}

QRectF Task::boundingRect() const
{
    return QRectF(0, 0, width, height);
}

void Task::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

	painter->fillRect(0,0, width, height, mycolor);

	// Highlight & shade
	painter->fillRect(0,0, width, 1, QColor(255,255,255,90));
	painter->fillRect(0,1, 1, height-1, QColor(255,255,255,90));

	painter->fillRect(0,height-1, width, 1, QColor(0,0,0,90));
	painter->fillRect(width - 1,0, 1, height, QColor(0,0,0,90));

	// Resize handles
	QColor shadeColor(0,0,0,100);
	painter->fillRect(0,0,5,height,shadeColor);
	painter->fillRect(width - 5,0,5,height,shadeColor);

	// Caption
	QString caption = node()->id.left(12);
	painter->drawText(boundingRect(), QString("%1").arg(caption), QTextOption(Qt::AlignVCenter | Qt::AlignHCenter));
}

QVariant Task::itemChange( GraphicsItemChange change, const QVariant & value )
{
	if (scene() && change == ItemPositionChange && !isResizing) 
	{
		QPointF newPos = value.toPointF();

		newPos.setX(qMax(0.0,newPos.x()));
		newPos.setY(this->y());

		start = newPos.x();
		currentTime = 0;

		return newPos;
	}
	return QGraphicsItem::itemChange(change, value);
}

void Task::setLength( int newLength )
{
	newLength = qMax(1,newLength);

	this->length = newLength;

	// Visual
	prepareGeometryChange();
	this->width = length;
}

void Task::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	if(isResizing)
	{
		if(resizeDir == 0)
			setLength(event->pos().x());
		else
		{
			int deltaX = event->pos().x() - clickPos.x();
			setPos(qMax(0.0,deltaX + myOldPos.x()), myOldPos.y());
			setLength(myOldWidth - deltaX);
		}

		event->accept();
		return;
	}

	QGraphicsItem::mouseMoveEvent(event);
}

void Task::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	if (event->button() == Qt::LeftButton)
	{
		clickPos = event->pos();
		myOldPos = pos();
		myOldWidth = width;

		int eventX = event->pos().x();

		int resizeRegion = 5;
		if(eventX < resizeRegion || eventX > width - resizeRegion)
		{
			resizeDir = (eventX > width - resizeRegion) ? 0 : 1;

			isResizing = true;
			event->accept();
			return;
		}
		//event->accept();
	}

	QGraphicsItem::mousePressEvent(event);
}

void Task::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	event->accept();
	isResizing = false;

	QGraphicsItem::mouseReleaseEvent(event);
}

void Task::drawDebug()
{
	glColorQt(QColor(60,220,100));
	glPointSize(15);
	glDisable(GL_LIGHTING);

	glBegin(GL_POINTS);
	foreach(Vector3 p, debugPoints)	glVector3(p);
	glColorQt(QColor(220,100,60));
	foreach(Vector3 p, debugPoints2) glVector3(p);
	glEnd();

	glEnable(GL_LIGHTING);
}

Structure::Node * Task::node()
{
	return active->getNode(this->property["nodeID"].toString());
}

bool Task::stillWorking()
{
	return currentTime < start + length;
}

void Task::reset()
{
	isReady = false;
	currentTime = start;
}

int Task::endTime()
{
	return start + length;
}

double Task::localT( int globalTime )
{
	double t = 0;

	if(globalTime >= start)
	{
		t = qMin( 1.0, double(globalTime - start) / double(length) );
	}
	else
	{
		t = -1;
	}

	return t;
}

void Task::setStart( int newStart )
{
	start = newStart;
	currentTime = 0;
	setX(newStart);
}

Structure::Node * Task::targetNode()
{
	if(!node()->property.contains("correspond")) return NULL;
	return target->getNode( node()->property["correspond"].toString() );
}

Structure::Curve * Task::targetCurve()
{
	Structure::Node* n = targetNode();
	if(!n || n->type() != Structure::CURVE) return NULL;
	return (Structure::Curve *)n;
}

Structure::Sheet * Task::targetSheet()
{
	Structure::Node* n = targetNode();
	if(!n || n->type() != Structure::SHEET) return NULL;
	return (Structure::Sheet *)n;
}

QVector< GraphDistance::PathPointPair > Task::weldPath( QVector< GraphDistance::PathPointPair > oldPath )
{
	QVector< GraphDistance::PathPointPair > cleanPath;

	// Find spatial position
	std::vector<Vec3d> spatialPath;
	QSet<int> allPath;
	foreach(GraphDistance::PathPointPair nc, oldPath) 
	{
		spatialPath.push_back( nc.position(active) );
		allPath.insert( allPath.size() );
	}

	std::vector<size_t> xrefs;
	weld(spatialPath, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());

	QSet<int> goodPath; 
    for(int i = 0; i < (int)xrefs.size(); i++)
	{
		oldPath[ xrefs[i] ] = oldPath[i];
		goodPath.insert( xrefs[i] );
	}

	foreach(int i, goodPath) cleanPath.push_back( oldPath[i] );
	
	return cleanPath;
}

bool Task::isActive( double t )
{
	return (t >= 0.0 && !isDone);
}

// Helper functions
NodeCoord Task::futureOtherNodeCoord( Structure::Link *link )
{
	Structure::Node *tn = targetNode();

	QString tlinkID = link->property["correspond"].toString();
	Structure::Link * tlink = target->getEdge(tlinkID);

	Structure::Node * tfutureOther = tlink->otherNode(tn->id);
	QString futureOtherID = tfutureOther->property["correspond"].toString();

	Vec4d futureOtherCoord = tlink->getCoordOther(tn->id).front();

	return qMakePair(futureOtherID, futureOtherCoord);
}

NodeCoord Task::futureLinkCoord( Structure::Link *link )
{
	NodeCoord fnc = futureOtherNodeCoord(link);
	Structure::Node * n = active->getNode(fnc.first);

	// Special case when connected to null set
	if(n->property.contains("nullSet") && !n->property["taskIsDone"].toBool())
	{
		Structure::Node * otherNode = link->otherNode( node()->id );
		Structure::Link * bestEdge = NULL;

		QVector<Structure::Node*> nullSet = active->nodesWithProperty("nullSet", n->property["nullSet"]);

		foreach(Structure::Node * nj, nullSet){
			if( bestEdge = active->getEdge(nj->id, otherNode->id) )
				break;
		}

		if( bestEdge ){
			fnc = NodeCoord( otherNode->id, bestEdge->getCoord(otherNode->id).front() );
		}
	}

	return fnc;
}

void Task::copyTargetEdge( Structure::Link *tlink )
{
	Structure::Node *n = node();
	Structure::Node *tn = targetNode();

	std::vector<Vec4d> coord = tlink->getCoord(tn->id);

	Structure::Node *tOther = tlink->otherNode(tn->id);
	QString otherID = tOther->property["correspond"].toString();
	Structure::Node *nOther = active->getNode(otherID);
	std::vector<Vec4d> coordOther = tlink->getCoordOther(tn->id);

	active->addEdge(n, nOther, coord, coordOther);
}

void Task::geometryMorph( double t )
{
	if(t < 0.0 || t > 1.0) return;

	// Geometry morph:
	if(node()->property.contains("samples"))
	{
		QVector<Vec3d> points, normals;

		if(node()->type() == Structure::CURVE)
		{	
			Structure::Curve * tcurve = (Structure::Curve *)targetNode();
			Structure::Curve * curve = (Structure::Curve *)node();

			Synthesizer::blendGeometryCurves(curve, tcurve, t, points, normals);
		}

		if(node()->type() == Structure::SHEET)
		{	
			Structure::Sheet * tsheet = (Structure::Sheet *)targetNode();
			Structure::Sheet * sheet = (Structure::Sheet *)node();

			Synthesizer::blendGeometrySheets(sheet, tsheet, t, points, normals);
		}

		node()->property["cached_points"].setValue(points);
		node()->property["cached_normals"].setValue(normals);
	}
}

// PREPARE
void Task::prepare()
{
	if ( isReady ) return;

	this->start = this->x();
	this->currentTime = start;
	this->isDone = false;

	this->property["orgCtrlPoints"].setValue( node()->controlPoints() );

	QVector<QString> runningTasks = active->property["activeTasks"].value< QVector<QString> >();
	foreach(QString id, runningTasks) qDebug() << id;
	qDebug() << "---";

	if (node()->type() == Structure::CURVE)
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
	else
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

	this->isReady = true;
	node()->property["isReady"] = true;
}

QVector<Structure::Link*> Task::filterEdges( Structure::Node * n, QVector<Structure::Link*> allEdges )
{
	QVector<Structure::Link*> edges;

	for(int i = 0; i < (int)allEdges.size(); i++){
		Structure::Node * otherI = allEdges[i]->otherNode(n->id);

		for(int j = 0; j < (int)edges.size(); j++){
			Structure::Node * otherJ = edges[j]->otherNode(n->id);
			double sumV = sumvec( otherI->geometricDiff(otherJ) ).norm();

			if(sumV < 1e-10)
			{
				otherI = NULL;
				break;
			}
		}

		// Skip edges not yet made
		if( (!otherI) || (otherI->property["taskType"] == Task::GROW && !otherI->property.contains("taskIsDone")) )
			continue;

		if(otherI) edges.push_back(allEdges[i]);
	}

	// Bin edges by their coordinates into 4 locations (2 for curves)
	QMap< int, QVector<Structure::Link*> > bin;
	foreach(Structure::Link * l, edges){
		Vec4d coord = l->getCoord(n->id).front();
		int idx = coord[0] > 0.5 ? (coord[1] > 0.5 ? 3 : 1) : (coord[1] > 0.5 ? 2 : 0);
		bin[idx].push_back(l);
	}

	edges.clear();

	//if(n->type() == Structure::SHEET && bin.keys().contains(0) && bin.keys().contains(3))
	//{
	//	// Get edges at furthest corners of sheet
	//	edges.push_back( bin[0].front() );
	//	edges.push_back( bin[3].front() );
	//}
	//else
	//{
		foreach( int i, bin.keys() ){
			QVector<Structure::Link*> similarEdges = bin[i];

			// Arbitrary choice for now..
			Structure::Link * furthest = similarEdges.front();

			if(edges.size() < 2) edges.push_back( furthest );
		}
	//}

	// Remove any shrunken edges
	QVector<Structure::Link*> afterShrink;
	if(edges.size() > 1)
	{
		foreach(Structure::Link * l, edges){
			if(!l->hasNodeProperty("shrunk", true))
				afterShrink.push_back(l);
		}

		edges = afterShrink;
	}

	return edges;
}

Structure::Link * Task::preferredEnd(Structure::Node * n, QVector<Structure::Link*> edges, Structure::Graph * g)
{
	// Pick end with more valence
	int maxValence = 0;
	Structure::Link * preferredLink = edges.front();
	foreach(Structure::Link* edge, edges){
		Structure::Node * otherNode = edge->otherNode( n->id );
		int curValence = g->valence(otherNode);
		if(curValence > maxValence){
			maxValence = curValence;
			preferredLink = edge;
		}
	}
	return preferredLink;
}

void Task::prepareShrinkCurveOneEdge( Structure::Link* l )
{
	Structure::Node * n = node();
	Structure::Curve* structure_curve = ((Structure::Curve*)n);

	Structure::Node * base = l->otherNode(n->id);

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

void Task::prepareShrinkCurve()
{
	Structure::Node * n = node();
	QVector<Structure::Link*> edges = filterEdges( n, active->getEdges(n->id) );
	Structure::Curve* structure_curve = ((Structure::Curve*)n);

	if(edges.size() == 1)
	{
		prepareShrinkCurveOneEdge( edges.front() );
	}

	if(edges.size() == 2)
	{
		// Links and positions (on myself)
		Structure::Link * linkA = edges.front();
		Structure::Link * linkB = edges.back();
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
		RMF rmf( positionalPath(pathA, 3) );
		if(!rmf.count()) return;

		property["rmf"].setValue( rmf );
		Vector3 X = rmf.U.front().r, Y = rmf.U.front().s, Z = rmf.U.front().t;

		// Curve encoded, to decode you need two points A,B and a frame XYZ
		property["cpCoords"].setValue( encodeCurve((Structure::Curve*)n, pointA, pointB, X,Y,Z) );

		// DEBUG:
		node()->property["rmf"].setValue( rmf );
		node()->property["rmf2"].setValue( RMF ( positionalPath(pathB, 3) ) );
	}
}

void Task::prepareGrowCurve()
{
	Structure::Node * n = node();
	Structure::Node * tn = targetNode();
	QVector<Structure::Link*> all_tedges = target->getEdges(tn->id);
	Structure::Curve * structure_curve = (Structure::Curve *)n;

	// Do not consider edges with non-ready nodes
	QVector<Structure::Link*> tedges;
	foreach(Structure::Link* edge, all_tedges){
		Structure::Node * tother = edge->otherNode( tn->id );
		Structure::Node * other = active->getNode( tother->property["correspond"].toString() );
		if( other->property.contains("taskIsDone") )
			tedges.push_back(edge);
	}
	
	if( all_tedges.empty() ) return;

	// Cut nodes grow case
	if( tedges.isEmpty() )
	{
		tedges.push_back( preferredEnd(tn, all_tedges, target) );
	}

	if (tedges.size() == 1)
	{
		Structure::Link * tlink = tedges.front();
		Structure::Node * tbase = tlink->otherNode(tn->id);

		Vec4d coordBase = tlink->getCoord(tbase->id).front();
		Vec4d coordSelf = tlink->getCoord(tn->id).front();

		QString baseID = tbase->property["correspond"].toString();
		Structure::Node* base = active->getNode(baseID);

		Vector3 linkPositionBase = base->position(coordBase);
		int cpIDX = structure_curve->controlPointIndexFromCoord( coordSelf );

		/// Place curve:

		// Make origin the position on me in which I will grow from
		structure_curve->moveBy( -n->position(coordSelf)  );

		Structure::Node * tn = target->getNode(n->property["correspond"].toString());
		Vec3d endDelta = tlink->position(tn->id) - tlink->positionOther(tn->id);
		QString corrBaseNodeID = tlink->otherNode(tn->id)->property["correspond"].toString();
		Vec3d posOther = active->getNode(corrBaseNodeID)->position(tlink->getCoordOther(tn->id).front());

		// Move curve to start point computed by looking at target geometry
		structure_curve->moveBy( posOther + endDelta );

		// Curve folding
		Array1D_Vector3 deltas = structure_curve->foldTo( coordSelf, true );

		// Growing instructions
		property["deltas"].setValue( deltas );
		property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );
	}

	if (tedges.size() == 2)
	{
		// Links, nodes and positions on the TARGET
		Structure::Link *tlinkA = tedges.front();
		Structure::Link *tlinkB = tedges.back();
		Structure::Node *totherA = tlinkA->otherNode( tn->id );
		Structure::Node *totherB = tlinkB->otherNode( tn->id );
		Vec4d othercoordA = tlinkA->getCoord(totherA->id).front();
		Vec4d othercoordB = tlinkB->getCoord(totherB->id).front();

		// Corresponding stuff on ACTIVE
		Structure::Node *otherA = active->getNode( totherA->property["correspond"].toString() );
		Structure::Node *otherB = active->getNode( totherB->property["correspond"].toString() );
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
		Node * auxA = new Structure::Curve(NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, pointA + endDeltaA ) ), "auxA_" + n->id);
		Node * auxB = new Structure::Curve(NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, pointB + endDeltaB ) ), "auxB_" + n->id);
		active->aux_nodes.push_back( auxA );
		active->aux_nodes.push_back( auxB );
		pathA = smoothEnd(auxA, Vec4d(0), pathA);
		pathB = smoothEnd(auxB, Vec4d(0), pathB);

		property["pathA"].setValue( pathA );
		property["pathB"].setValue( pathB );
		
		// Encode curve
		RMF rmf( positionalPath(pathA, 3) );
		property["rmf"].setValue( rmf );
		if(!rmf.count()) return;

		Vector3 X = rmf.U.back().r, Y = rmf.U.back().s, Z = rmf.U.back().t;
		property["cpCoords"].setValue( encodeCurve((Structure::Curve*)tn, tlinkA->position(tn->id), tlinkB->position(tn->id), X,Y,Z) );

		// DEBUG frames
		node()->property["rmf"].setValue( rmf );
		node()->property["rmf2"].setValue( RMF ( positionalPath(pathB, 3) ) );

		// Initial position of curve node
        Vec4d midCoord(0.5);
        structure_curve->foldTo(midCoord, true);
        structure_curve->curve.translate(startPoint - structure_curve->position(midCoord));
	}
}

/* Curve encoding, to decode you need two points A,B and a frame XYZ */
CurveEncoding Task::encodeCurve( Array1D_Vector3 points, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z, bool isFlip )
{
	CurveEncoding cpCoords;

	Line segment(start, end);

	for(int i = 0; i < (int)points.size(); i++)
	{
		double t;
		Vector3 p = points[i], proj;
		segment.ClosestPoint(p, t, proj);

		Vector3 dir = p - proj;

		// Parameters: t, offset, theta, psi
		Array1D_Real params(4,0);
		params[0] = t;
		if(dir.norm() > 0){
			params[1] = dir.norm() / segment.length;
			dir = dir.normalized();
		}
		globalToLocalSpherical(X,Y,Z, params[2], params[3], dir);

		// Flipping case
		int idx = i;
		if(isFlip) idx = (points.size()-1) - i; 

		cpCoords[idx] = params;
	}

	return cpCoords;
}

CurveEncoding Task::encodeCurve( Structure::Curve * curve, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z, bool isFlip )
{
	return encodeCurve(curve->controlPoints(),start,end,X,Y,Z, isFlip);
}

Array1D_Vector3 Task::decodeCurve(CurveEncoding cpCoords, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z, double T)
{
	Array1D_Vector3 controlPoints (cpCoords.size(), Vector3(0));

	Line segment(start, end);

	for(int i = 0; i < (int)controlPoints.size(); i++)
	{
		double t = cpCoords[i][0];
		double offset = cpCoords[i][1];
		double theta = cpCoords[i][2];
		double psi = cpCoords[i][3];

		Vector3 dir(0);
		localSphericalToGlobal(X,Y,Z,theta,psi,dir);
	
		controlPoints[i] = segment.pointAt(t) + (dir * ((offset * T) * segment.length));
	}

	return controlPoints;
}

void Task::prepareMorphCurve()
{
	Structure::Node * n = node();
	Structure::Node * tn = targetNode();

	Structure::Curve * curve = (Structure::Curve *)n;
	Structure::Curve * tcurve = (Structure::Curve *)tn;
	
	// 0) Filter edges (remove edges with null since these nodes will grow in future)
	QVector<Structure::Link*> edges = filterEdges(n, active->getEdges(n->id));
	QVector<Structure::Link*> tedges = target->getEdges(tn->id);

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		Structure::Link * link = edges.front();
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
			path = smoothStart( node(), link->getCoord(n->id).front(), path );
		else
		{
			// Add a "static" point
			path.clear();
			Node * aux = addAuxNode(link->position(n->id), active);
			path.push_back( GraphDistance::PathPointPair( PathPoint (aux->id, Vec4d(0)) ) );
		}

		// Smooth transition to end point
		path = smoothEnd( prepareEnd(n, link), Vec4d(0), path );

		// Replace coordinates and such
		link->replace( otherNode, active->getNode(fnc.first), std::vector<Vec4d>(1,fnc.second) );

		path = weldPath( path );

		property["path"].setValue( path );

		// Consistent frames using path
		RMF rmf( positionalPath(path, 2) );
		property["rmf"].setValue( rmf );
		Vector3 X = rmf.U.back().r, Y = rmf.U.back().s, Z = rmf.U.back().t;

		// Encode source curve
		Vec3d startA = startPoint;
		Vec3d startB = n->position( coordB );
		property["cpCoords"].setValue( encodeCurve((Structure::Curve*)n, startA, startB, X,Y,Z) );
		property["endPointDelta"].setValue( startB - startA );

		// Encode target curve
		Vec3d tstartA = tn->position( coordA );
		Vec3d tstartB = tn->position( coordB );
		property["cpCoordsT"].setValue( encodeCurve((Structure::Curve*)tn, tstartA, tstartB, X,Y,Z) );
		property["endPointDeltaT"].setValue( tstartB - tstartA );

		// DEBUG:
		node()->property["path"].setValue( positionalPath(path) );
	}

	// 2) TWO edges
	if(edges.size() == 2)
	{
		// Start and end for both links
		Structure::Link * linkA = edges.front();
		Vec3d startA = linkA->position(n->id);
		NodeCoord edA = futureLinkCoord(linkA);
		Vec3d endA = active->position(edA.first,edA.second);

		Structure::Link * linkB = edges.back();
		Vec3d startB = linkB->position(n->id);
		NodeCoord edB = futureLinkCoord(linkB);
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
			else 
				pathA = smoothStart(node(), linkA->getCoord(n->id).front(), pathA);

			if( isPathOnSingleNode(pathB) )	
			{
				// Add a "static" point
				pathB.clear();
				Node * aux = addAuxNode(linkB->position(n->id), active);
				pathB.push_back( GraphDistance::PathPointPair( PathPoint (aux->id, Vec4d(0)) ) );
			}
			else 
				pathB = smoothStart(node(), linkB->getCoord(n->id).front(), pathB);
		}

		// Replace destination
		foreach(Structure::Link *link, edges){
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
		
		// Smooth transition to end point
		QPair<Node*,Node*> auxAB = prepareEnd2(n,linkA,linkB);
		pathA = smoothEnd(auxAB.first, Vec4d(0), pathA);
		pathB = smoothEnd(auxAB.second, Vec4d(0), pathB);

		// Remove redundancy along paths
		pathA = this->weldPath( pathA );
		pathB = this->weldPath( pathB );

		property["pathA"].setValue( pathA );
		property["pathB"].setValue( pathB );

		// Corresponding links
		Structure::Link * tlinkA = target->getEdge(linkA->property["correspond"].toString());
		Structure::Link * tlinkB = target->getEdge(linkB->property["correspond"].toString());
		Vec3d tstartA = tlinkA->position(tn->id);
		Vec3d tstartB = tlinkB->position(tn->id);

		// Check for reversed coordinates
		bool isFlip = false;
		bool isFlipA = false, isFlipB = false;
		if( !isSameHalf(linkA->getCoord(n->id).front(), tlinkA->getCoord(tn->id).front()) )	isFlipA = true;
		if( !isSameHalf(linkB->getCoord(n->id).front(), tlinkB->getCoord(tn->id).front()) )	isFlipB = true;
		isFlip = isFlipA && isFlipB;

		// Get source and target frames
		RMF::Frame sframe = curveFrame( curve );
		RMF::Frame tframe = curveFrame( tcurve );

		// Compute rotations between source and target sheet frames
		Eigen::Quaterniond rotation;
		AbsoluteOrientation::compute(sframe.r,sframe.s,sframe.t, 
									 tframe.r,tframe.s,tframe.t, rotation);

		// Parameters needed for morphing
		property["rotation"].setValue( rotation );
		property["sframe"].setValue( sframe );
		property["tframe"].setValue( tframe );

		property["cpCoords"].setValue( encodeCurve((Structure::Curve*)n, startA, startB, sframe.r,sframe.s,sframe.t) );
		property["cpCoordsT"].setValue( encodeCurve((Structure::Curve*)tn, tstartA, tstartB, tframe.r,tframe.s,tframe.t, isFlip) );

		property["isFlip"].setValue( isFlip );

		// Visualization
		n->property["frame"].setValue( sframe );
		tn->property["frame"].setValue( tframe );
		n->property["rmf"].setValue( RMF(positionalPath(pathA,2)) );
		n->property["path"].setValue( positionalPath(pathB) );
	}
}

SheetEncoding Task::encodeSheetAsCurve( Structure::Sheet * sheet, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z )
{
	Array1D_Vector3 controlPoints = sheet->controlPoints();
	return encodeCurve(controlPoints,start,end,X,Y,Z);
}

Array1D_Vector3 Task::decodeSheetFromCurve( double t, SheetEncoding cpCoords, Vector3 start, Vector3 end, Vector3 X, Vector3 Y, Vector3 Z )
{
	return decodeCurve(cpCoords,start,end,X,Y,Z,t);
}

void Task::prepareSheetOneEdge( Structure::Link * l )
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

void Task::prepareSheetTwoEdges( Structure::Link * linkA, Structure::Link * linkB )
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
	RMF rmf( positionalPath(pathA, 3) );
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

void Task::prepareGrowShrinkSheet()
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

void Task::prepareMorphSheet()
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

RMF::Frame Task::sheetFrame( Structure::Sheet * sheet )
{
	Vector3 corner = sheet->position(Vec4d(0));
	Vector3 A = sheet->position(Vec4d(0,1,0,0));
	Vector3 B = sheet->position(Vec4d(1,0,0,0));
	Vector3 C = sheet->position(Vec4d(1));

	Vector3 X = (A - corner).normalized();
	Vector3 Y = (B - corner).normalized();
	Vector3 Z = cross(X,Y);

	RMF::Frame frame = RMF::Frame::fromTR(Z,X);
	frame.center = (A + B + C + corner) / 4.0;
	return frame;
}

Array1D_Vector3 Task::sheetDeltas( Structure::Sheet * sheet )
{
	Array1D_Vector3 deltas;
	RMF::Frame frame = sheetFrame(sheet);
	Array1D_Vector3 controlPoints = sheet->controlPoints();
	for(int i = 0; i < (int)controlPoints.size(); i++)
		deltas.push_back( controlPoints[i] - frame.center );
	return deltas;
}

SheetEncoding Task::encodeSheet( Structure::Sheet * sheet, Vector3 origin, Vector3 X, Vector3 Y, Vector3 Z )
{
	SheetEncoding cpCoords;
	Array1D_Vector3 controlPoints = sheet->controlPoints();
	
	for(int i = 0; i < (int)controlPoints.size(); i++)
	{
		Vector3 p = controlPoints[i];
		Vector3 dir = p - origin;

		// Parameters: offset, theta, psi
		Array1D_Real params(3,0);
		if(dir.norm() > 0){
			params[0] = dir.norm();
			dir = dir.normalized();
		}
		globalToLocalSpherical(X,Y,Z, params[1], params[2], dir);

		cpCoords[i] = params;
	}

	return cpCoords;
}

Array1D_Vector3 Task::decodeSheet( SheetEncoding cpCoords, Vector3 origin, Vector3 X, Vector3 Y, Vector3 Z )
{
	Array1D_Vector3 pnts( cpCoords.size(), Vector3(0) );

	for(int i = 0; i < (int)pnts.size(); i++)
	{
		double offset = cpCoords[i][0];
		double theta = cpCoords[i][1];
		double psi = cpCoords[i][2];

		Vector3 dir(0);
		localSphericalToGlobal(X,Y,Z,theta,psi,dir);

		pnts[i] = origin + (dir * offset);
	}

	return pnts;
}

RMF::Frame Task::curveFrame( Structure::Curve * curve, bool isFlip )
{
	Vec4d zero(0);
	Vec4d one(1.0);

	if(isFlip) std::swap(zero,one);

	Vec3d origin = curve->position(zero);
	Vec3d X = (curve->position(one) - origin).normalized();
	Vec3d Y = orthogonalVector(X);
	RMF::Frame frame = RMF::Frame::fromRS(X,Y);
	frame.center = origin; 
	return frame;
}

// EXECUTE
void Task::execute( double t )
{	
	if( !isActive(t) ) return;

	currentTime = start + (t * length);

	// Range check
	t = qRanged(0.0, t, 1.0);

	// Execute curve tasks
	if (node()->type() == Structure::CURVE)
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
	
	// Execute sheet tasks
	if (node()->type() == Structure::SHEET)
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

	// Post execution steps
	if(t >= 1.0)
	{
		Structure::Node * n = node();
		Structure::Node * tn = targetNode();

		this->isDone = true;
		n->property["taskIsDone"] = true;

		if(type == SHRINK)
		{
			n->property["shrunk"] = true;
		}

		/*if(type == GROW && n->property["isCutGroup"].toBool())
		{
			QVector<Structure::Link*> edges = active->getEdges(n->id);
			QVector<Structure::Link*> tedges = target->getEdges(tn->id);

			// Re-assign source edges based on target counterparts
			QVector<Node*> tadjN;
			active->removeEdges( n->id );
			foreach(Structure::Link * l, tedges){
				Node * n1 = active->getNode(l->n1->property["correspond"].toString());
				Node * n2 = active->getNode(l->n2->property["correspond"].toString());
				Array1D_Vec4d coord1 = l->getCoord(l->n1->id);
				Array1D_Vec4d coord2 = l->getCoord(l->n2->id);
				Link * newEdge = active->addEdge(n1,n2,coord1,coord2);
				newEdge->property["correspond"] = l->id;
				
				if(l->n1 != tn) tadjN.push_back(l->n1);
				if(l->n2 != tn) tadjN.push_back(l->n2);
			}
			
			// Make a copy of target graph and cut current target node
			Structure::Graph targetCopy( *target );
			targetCopy.removeNode( tn->id );
			
			// Search if paths are not valid then disconnect at source graph
			for(int i = 0; i < (int)tadjN.size(); i++){
				for(int j = i + 1; j < (int)tadjN.size(); j++){
					QVector<Node*> path = targetCopy.path(tadjN[i], tadjN[j]);
					if( path.size() < 2 ){
						Node * n1 = active->getNode(tadjN[i]->property["correspond"].toString());
						Node * n2 = active->getNode(tadjN[j]->property["correspond"].toString());
						active->removeEdge(n1->id, n2->id);
					}
				}
			}
		}*/
	}

	// Special cases
	if(t >= 0)
	{
		if(type == GROW)
		{
			node()->property["toGrow"] = false;
		}
	}
}

void Task::executeGrowShrinkCurve( double t )
{
	Structure::Node *n = node();
	
	// Regular shrink
	if (property.contains("deltas"))
		foldCurve(t);
	else if (property.contains("pathA") && property.contains("pathB"))
		executeMorphCurve(t);
	
	// When task is done
	if (t == 1)
	{	
		QVector<Structure::Link*> edges = active->getEdges(n->id);
		
		if(type == GROW)
		{

		}

		if(type == SHRINK)
		{
			n->property["isReady"] = false;

			// Delete all edges
			foreach(Structure::Link *link, edges){
				if(link->property.contains("changingEnd")) 
					continue;
				//active->removeEdge(link->n1, link->n2);
			}

			//Structure::Curve* structure_curve = ((Structure::Curve*)node());
			//structure_curve->foldTo(Vec4d(0),true);
		}
	}
}

void Task::foldCurve( double t )
{
	Structure::Curve* structure_curve = ((Structure::Curve*)node());

	Array1D_Vector3 cpts = property["orgCtrlPoints"].value<Array1D_Vector3>();
	Array1D_Vector3 deltas = property["deltas"].value<Array1D_Vector3>();

	if(cpts.size() != deltas.size()) return;

	// Grow curve
	for(int u = 0; u < structure_curve->curve.mNumCtrlPoints; u++)
		structure_curve->curve.mCtrlPoint[u] = cpts[u] + (deltas[u] * t);	
}

void Task::executeGrowShrinkSheet( double t )
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
		double decodeT = qMin(1.0, t * 2.0);

		if(this->type == SHRINK) dt = 1 - t;

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
			foreach(Structure::Link *link, edges)
			{
				//active->removeEdge(link->n1, link->n2);
			}
		}

		if (type == GROW)
		{
			if(tedges.size()) copyTargetEdge(tedges.front());
		}
	}
}

void Task::executeMorphCurve( double t )
{
	Structure::Node * n = node();
	Structure::Node * tn = targetNode();

	Structure::Curve* structure_curve = ((Structure::Curve*)n);

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
		RMF rmf = property["rmf"].value<RMF>();
		
		int idx = t * (path.size() - 1);

		Vec3d newHandlePos = path[idx].position(active); // Other end = newHandlePos + endPointDelta

		Vector3 X = rmf.frameAt(t).r, Y = rmf.frameAt(t).s, Z = rmf.frameAt(t).t;

		// Decode and blend
		Array1D_Vector3 newPnts = decodeCurve( cpCoords, newHandlePos, newHandlePos + endPointDelta, X, Y, Z );
		Array1D_Vector3 newPntsT = decodeCurve( cpCoordsT, newHandlePos, newHandlePos + endPointDeltaT, X, Y, Z );
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

		int idxA = t * (pathA.size() - 1);
		int idxB = t * (pathB.size() - 1);

		// Move to next step
		Vector3 pointA = pathA[idxA].position(active);
		Vector3 pointB = pathB[idxB].position(active);

		// Decode
		SheetEncoding cpCoords = property["cpCoords"].value<SheetEncoding>();
		SheetEncoding cpCoordsT = property["cpCoordsT"].value<SheetEncoding>();

		// Frames
		RMF::Frame sframe = property["sframe"].value<RMF::Frame>();
		RMF::Frame tframe = property["tframe"].value<RMF::Frame>();
		Eigen::Quaterniond rotation = property["rotation"].value<Eigen::Quaterniond>(), 
			eye = Eigen::Quaterniond::Identity();

		// Source curve
		Eigen::Vector3d R = V2E(sframe.r), S = V2E(sframe.s), T = V2E(sframe.t);
		R = eye.slerp(t, rotation) * R;
		S = eye.slerp(t, rotation) * S;
		T = eye.slerp(t, rotation) * T;
		RMF::Frame curFrame (E2V(R),E2V(S),E2V(T));

		curFrame.center = tframe.center = AlphaBlend(t, sframe.center, tframe.center);
		
		bool isFlip = property["isFlip"].toBool();

		Array1D_Vector3 newPnts = decodeCurve(property["cpCoords"].value<CurveEncoding>(), pointA, pointB, sframe.r,sframe.s,sframe.t);
		Array1D_Vector3 newPntsT = decodeCurve(property["cpCoordsT"].value<CurveEncoding>(), pointA, pointB, tframe.r,tframe.s,tframe.t);

		if(!newPntsT.size()) newPntsT = newPnts;

		Array1D_Vector3 blendedPnts;

		for(int i = 0; i < (int)newPnts.size(); i++)
		{
			blendedPnts.push_back( AlphaBlend(t, newPnts[i], newPntsT[i]) );
		}

		structure_curve->setControlPoints( blendedPnts );

		// DEBUG:
		//active->debugPoints.push_back( pointA );
		//active->debugPoints.push_back( pointB );
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
				Structure::Link *linkA = edges.front();
				Structure::Link *linkB = edges.back();

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

void Task::executeMorphSheet( double t )
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

Array1D_Vector3 Task::positionalPath( QVector< GraphDistance::PathPointPair > & from_path, int smoothingIters )
{
	Array1D_Vector3 pnts;
	if(!from_path.size()) return pnts;

	foreach(GraphDistance::PathPointPair p, from_path) 
		pnts.push_back(p.position(active));

	// To ensure unique points
	std::vector<size_t> xrefs;
	weld(pnts, xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());

	return smoothPolyline(pnts, smoothingIters);
}

QVector< GraphDistance::PathPointPair > Task::smoothStart( Structure::Node * n, Vec4d startOnNode, QVector< GraphDistance::PathPointPair > oldPath )
{
	if(!oldPath.size()) return oldPath;
	QVector< GraphDistance::PathPointPair > prefix_path;
	PathPoint a = PathPoint(n->id, startOnNode), b = oldPath.front().a;

	prefix_path.push_back( GraphDistance::PathPointPair(a) );

	Vector3 start = active->position(a.first, a.second);
	Vector3 end = active->position(b.first, b.second);
	double dist = (start - end).norm();

	// Add virtual path points if needed
	if(dist > DIST_RESOLUTION){
		int steps = dist / DIST_RESOLUTION;
		for(int i = 1; i < steps; i++){
			double alpha = double(i) / steps;
			prefix_path.push_back( GraphDistance::PathPointPair( a, b, alpha ) );
		}
	}

	return prefix_path + oldPath;
}

QVector< GraphDistance::PathPointPair > Task::smoothEnd( Structure::Node * n, Vec4d startOnNode, QVector< GraphDistance::PathPointPair > oldPath )
{
	if(!oldPath.size()) return oldPath;
	QVector< GraphDistance::PathPointPair > postfix_path;
	PathPoint a = oldPath.back().a, b = PathPoint(n->id, startOnNode);

	Vector3 start = active->position(a.first, a.second);
	Vector3 end = active->position(b.first, b.second);
	double dist = (start - end).norm();

	// Add virtual path points if needed
	if(dist > DIST_RESOLUTION){
		int steps = dist / DIST_RESOLUTION;
		for(int i = 1; i < steps; i++){
			double alpha = double(i) / steps;
			postfix_path.push_back( GraphDistance::PathPointPair( a, b, alpha ) );
		}
	}

	postfix_path.push_back( GraphDistance::PathPointPair( a, b, 1.0 ) );

	return oldPath + postfix_path;
}

bool Task::isPathOnSingleNode( QVector< GraphDistance::PathPointPair > path )
{
	QSet<QString> nodeIDs;

	foreach(GraphDistance::PathPointPair p, path)
		nodeIDs.insert( p.a.first );

	return nodeIDs.size() <= 1;
}

Structure::Link * Task::getCoorespondingEdge( Structure::Link * link, Structure::Graph * otherGraph )
{
	QString correspondID = link->property["correspond"].toString();
	return otherGraph->getEdge(correspondID);
}

Structure::Node * Task::addAuxNode(Vec3d position, Structure::Graph * g)
{
	Structure::Node * aux = new Structure::Curve(NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, position ) ), 
		"auxA_" + QString::number(g->aux_nodes.size()));
	g->aux_nodes.push_back( aux );
	return aux;
}

Structure::Node * Task::prepareEnd( Structure::Node * n, Structure::Link * slink )
{
	Structure::Node * tn = target->getNode(n->property["correspond"].toString());
	Structure::Link * tlink = target->getEdge(slink->property["correspond"].toString());
	Vec3d endDelta = tlink->position(tn->id) - tlink->positionOther(tn->id);
	QString corrBaseNodeID = tlink->otherNode(tn->id)->property["correspond"].toString();
	Vec3d posOther = active->getNode(corrBaseNodeID)->position(tlink->getCoordOther(tn->id).front());
	Structure::Node * aux = new Structure::Curve(NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, posOther + endDelta ) ), "auxA_" + n->id);
	active->aux_nodes.push_back( aux );
	return aux;
}

QPair<Structure::Node*,Structure::Node*> Task::prepareEnd2( Structure::Node * n, Structure::Link * linkA, Structure::Link * linkB )
{
	Structure::Node * tn = target->getNode(n->property["correspond"].toString());
	Structure::Link * tlinkA = target->getEdge(linkA->property["correspond"].toString());
	Structure::Link * tlinkB = target->getEdge(linkB->property["correspond"].toString());
	Vec3d endDeltaA = tlinkA->position(tn->id) - tlinkA->positionOther(tn->id);
	Vec3d endDeltaB = tlinkB->position(tn->id) - tlinkB->positionOther(tn->id);
	Node * auxA = new Structure::Curve(NURBSCurved::createCurveFromPoints( 
		Array1D_Vector3 ( 4, linkA->otherNode(n->id)->position(tlinkA->getCoordOther(tn->id).front()) + endDeltaA ) ), "auxA_" + n->id);
	Node * auxB = new Structure::Curve(NURBSCurved::createCurveFromPoints( 
		Array1D_Vector3 ( 4, linkB->otherNode(n->id)->position(tlinkB->getCoordOther(tn->id).front()) + endDeltaB ) ), "auxB_" + n->id);
	active->aux_nodes.push_back( auxA );
	active->aux_nodes.push_back( auxB );
	return qMakePair(auxA, auxB);
}

void Task::setNode( QString node_ID )
{
	property["nodeID"] = node_ID;
	this->nodeID = node_ID;
	node()->property["taskType"] = type;

	if(type == GROW)
	{
		node()->property["toGrow"] = true;
	}
}

std::vector<RMF::Frame> Task::smoothRotateFrame( RMF::Frame sframe, Eigen::Quaterniond & rotation, int steps )
{
	std::vector<RMF::Frame> frames;

	double stepSize = 1.0 / double(steps);

	Eigen::Quaterniond eye = Eigen::Quaterniond::Identity();

	for(double alpha = 0; alpha <= 1.0; alpha += stepSize)
	{
		Eigen::Vector3d r = V2E(sframe.r), s = V2E(sframe.s), t = V2E(sframe.t);
		r = eye.slerp(alpha, rotation) * r;
		s = eye.slerp(alpha, rotation) * s;
		t = eye.slerp(alpha, rotation) * t;

		RMF::Frame curFrame = RMF::Frame(E2V(r),E2V(s),E2V(t));
		curFrame.center = sframe.center;
		frames.push_back(curFrame);
	}
	return frames;
}
