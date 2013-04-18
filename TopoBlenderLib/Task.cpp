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
	
	node()->property["localT"] = t;
	active->property["targetGraph"].setValue( target );
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
        prepareCurve();
	}

    if (node()->type() == Structure::SHEET)
	{
        prepareSheet();
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
		int curValence = g->valence(otherNode) * (otherNode->type() == Structure::CURVE ? 1 : 100 );
		if(curValence > maxValence){
			maxValence = curValence;
			preferredLink = edge;
		}
	}
	return preferredLink;
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
        executeCurve(t);
	}
	
	// Execute sheet tasks
	if (node()->type() == Structure::SHEET)
	{
        executeSheet(t);
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

		// Case: done growing a cut node
		if(type == GROW && n->property["isCutGroup"].toBool())
		{
			// re-link adjacent nodes
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
		}
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

	//Vec3d fromOtherA = linkA->otherNode(n->id)->position(tlinkA->getCoordOther(tn->id).front());
	//Vec3d fromOtherB = linkB->otherNode(n->id)->position(tlinkB->getCoordOther(tn->id).front());

	Vec3d fromOtherA = tlinkA->position(tn->id) - endDeltaA;
	Vec3d fromOtherB = tlinkB->position(tn->id) - endDeltaB;

	Node * auxA = new Structure::Curve(NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, fromOtherA + endDeltaA ) ), "auxA_" + n->id);
	Node * auxB = new Structure::Curve(NURBSCurved::createCurveFromPoints( Array1D_Vector3 ( 4, fromOtherB + endDeltaB ) ), "auxB_" + n->id);

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
