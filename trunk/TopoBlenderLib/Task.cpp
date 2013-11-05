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
	this->length = Task::DEFAULT_LENGTH;

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
	// Glow when selected
	if( this->isSelected() )
	{
		QGraphicsDropShadowEffect * taskGlow = new QGraphicsDropShadowEffect;
		taskGlow->setColor(QColor(255,255,255,255));
		taskGlow->setBlurRadius(20);
		taskGlow->setOffset(0);
		this->setGraphicsEffect(taskGlow);

		if(node() && targetNode())
		{
			node()->vis_property["glow"] = true;
			targetNode()->vis_property["glow"] = true;
		}
	}
	else
	{
		this->setGraphicsEffect(0);

		if(node() && targetNode())
		{
			node()->vis_property["glow"] = false;
			targetNode()->vis_property["glow"] = false;
		}
	}
	
	if (change == ItemPositionChange && scene() && !isResizing) 
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

	if(!(event->modifiers() & Qt::ShiftModifier))
	{
		// Select siblings in the group, if non-else selected
		bool nothingElseSelected = true;
		foreach(Task * otherTask, allOtherTasks()){
			if(otherTask->isSelected()){
				nothingElseSelected = false;
				break;
			}
		}
		if( nothingElseSelected ){
			bool needRefresh = false;
			foreach(QString member, active->groupOf(nodeID)){
				Task * t = active->getNode(member)->property["task"].value<Task*>();

				if(abs(t->start - this->start) < 10){
					t->setSelected(true);
					needRefresh = true;
				}
			}
			if(needRefresh) ((Scheduler*)this->scene())->emitUpdateExternalViewer();
		}

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

	vs1.draw(3);
	vs2.draw(6);

	ls1.draw();
	ls2.draw();

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
	std::vector<Vector3d> spatialPath;
	QSet<int> allPath;
	foreach(GraphDistance::PathPointPair nc, oldPath) 
	{
		spatialPath.push_back( nc.position(active) );
		allPath.insert( allPath.size() );
	}

	std::vector<size_t> xrefs;
	weld(spatialPath, xrefs, std::hash_Vector3d(), std::equal_to<Vector3d>());

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
	Structure::Link * tlink = target->getEdge( link->property["correspond"].toInt() );

	Structure::Node * tfutureOther = tlink->otherNode(tn->id);
	QString futureOtherID = tfutureOther->property["correspond"].toString();

	Vector4d futureOtherCoord = tlink->getCoordOther(tn->id).front();

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

	// Initialize
	{
		this->start = this->x();
		this->currentTime = start;
		this->isDone = false;
		this->property["orgCtrlPoints"].setValue( node()->controlPoints() );
	}

	// Reset any modified edges
	{
		Node * n = node();
		if(n->property["edgesModified"].toBool()){
			foreach(Link * l, active->getEdges(n->id))
				l->popState();
		}
	}

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
	node()->property["taskIsReady"] = true;
}

QVector<Structure::Link*> Task::filterEdges( Structure::Node * n, QVector<Structure::Link*> allEdges )
{
	// Check edge's real ownership
	Node * tn = target->getNode(n->property["correspond"].toString());
	if( tn ){
		QVector<Link*> keepEdges;
		foreach(Link * sl, allEdges)
		{
			Link * tl = target->getEdge( sl->property["correspond"].toInt() );

			// Skip target edges that are not related to current node
			if(!tl->hasNode(tn->id))
				continue;
			else
				keepEdges.push_back(sl);
		}
		allEdges = keepEdges;
	}

	QVector<Structure::Link*> edges;

	for(int i = 0; i < (int)allEdges.size(); i++){
		Structure::Node * otherI = allEdges[i]->otherNode(n->id);

		for(int j = 0; j < (int)edges.size(); j++){
			Structure::Node * otherJ = edges[j]->otherNode(n->id);
			double sumV = sumvec( otherI->geometricDiff(otherJ) ).norm();

			if(sumV < 1e-10){
				otherI = NULL;
				break;
			}
		}

		// Skip edges not yet made
		if( (!otherI) || ungrownNode(otherI->id) ) continue;

		if(otherI) edges.push_back(allEdges[i]);
	}

	// Bin edges by their coordinates into 4 locations (2 for curves)
	QMap< int, QVector<Structure::Link*> > bin;
	foreach(Structure::Link * l, edges){
		Vector4d coord = l->getCoord(n->id).front();
		int idx = coord[0] > 0.5 ? (coord[1] > 0.5 ? 3 : 1) : (coord[1] > 0.5 ? 2 : 0);
		bin[idx].push_back(l);
	}

	edges.clear();

	foreach( int i, bin.keys() ){
		QVector<Structure::Link*> similarEdges = bin[i];

		// Arbitrary choice for now..
		Structure::Link * furthest = similarEdges.front();

		if(edges.size() < 2) edges.push_back( furthest );
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


RMF::Frame Task::sheetFrame( Structure::Sheet * sheet )
{
	Vector3 corner = sheet->position(Vector4d(0,0,0,0));
	Vector3 A = sheet->position(Vector4d(0,1,0,0));
	Vector3 B = sheet->position(Vector4d(1,0,0,0));
	Vector3 C = sheet->position(Vector4d(1,1,1,1));

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

RMF::Frame Task::curveFrame( Structure::Curve * curve, bool isFlip )
{
	Vector4d zero(0,0,0,0);
	Vector4d one(1,1,1,1);
	if(isFlip) std::swap(zero,one);
	Vector3d origin = curve->position(zero);
	Vector3d X = (curve->position(one) - origin).normalized();
	Vector3d Y = orthogonalVector(X);
	RMF::Frame frame = RMF::Frame::fromRS(X,Y);
	frame.center = origin; 
	return frame;
}

// EXECUTE
void Task::execute( double t )
{	
	if( !isActive(t) ) return;

	// Range check
	t = qRanged(0.0, t, 1.0);

	currentTime = start + (t * length);

	// Make available for reconstruction
	if( t > 0.0 ) node()->property["zeroGeometry"] = false;

	// Execute task by type
	if (node()->type() == Structure::CURVE)	executeCurve( t );
	if (node()->type() == Structure::SHEET)	executeSheet( t );
	
	// Fix the other end of all edges
	executeMorphEdges( t );

	// Post execution steps
	if(t >= 1.0) postDone();

	// Record current time
	property["t"] = t;
	node()->property["t"] = t;
}

void Task::postDone()
{
	Structure::Node * n = node();

	this->isDone = true;
	n->property["taskIsDone"] = true;

	// Remove paths for edges
	QVector<Structure::Link*> edges = property["edges"].value< QVector<Structure::Link*> >();
	foreach (Structure::Link* link, edges){
		QVector< GraphDistance::PathPointPair > path = link->property["path"].value< QVector< GraphDistance::PathPointPair > >();
		if(path.size()) 
			link->property.remove("path");
	}

	// Post shrinking
	if(type == SHRINK)
	{
		n->property["shrunk"] = true;
		n->property["zeroGeometry"] = true;

		// remove all edges for non cutting node after shrunk
		if (!isCutting())
		{
			foreach(Link* link, active->getEdges(nodeID))
			{
				active->removeEdge(link->n1, link->n2);
			}
		}
	}

	// Post merging
	if(n->property["taskTypeReal"].toInt() == Task::MERGE)
	{
		// Disconnect other merged nodes
		foreach(QString siblingID, active->groupOf(nodeID))
		{
			Structure::Node * sibling = active->getNode(siblingID);

			// Ignore myself, non-merging, and already dealt with
			if(siblingID == nodeID || !sibling->property["taskTypeReal"].toInt() == Task::MERGE) continue;
			if(sibling->property["merged"].toBool()) continue;

			// Transfer edges
			foreach(Link* link, active->getEdges(siblingID))
			{
				QString neighbour = link->otherNode(siblingID)->id;
				if(active->getEdge(nodeID, neighbour)) 
				{
					// Remove shared ones
					active->removeEdge(siblingID, neighbour);
				}
				else
				{
					// Replace to node of this task
					link->replace(siblingID, n, link->getCoord(siblingID));
				}
			}

			// "remove" the node from execution
			sibling->property["shrunk"] = true;
			sibling->property["zeroGeometry"] = true;
			sibling->property["taskIsDone"] = true;

			Task * siblingTask = sibling->property["task"].value<Task*>();
			siblingTask->isDone = true;
			siblingTask->property.remove("edges");
		}

		// Fix coordinates of edges by looking at the target
		foreach(Link* link, active->getEdges(nodeID))
		{
			Link * tlink = target->getEdge(link->property["correspond"].toInt());
			Array1D_Vector4d coordMe = tlink->getCoord(n->property["correspond"].toString());
			Array1D_Vector4d coordOther = tlink->getCoordOther(n->property["correspond"].toString());

			link->setCoord(nodeID, coordMe);
			link->setCoord(link->otherNode(nodeID)->id, coordOther);
		}

		n->property["merged"] = true;
	}

	// Clean up:
	n->property.remove("path");
	n->property.remove("path2");
}

QVector< GraphDistance::PathPointPair > Task::smoothStart( Structure::Node * n, Vector4d& startOnNode, QVector< GraphDistance::PathPointPair > oldPath )
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

QVector< GraphDistance::PathPointPair > Task::smoothEnd( Structure::Node * n, Vector4d& startOnNode, QVector< GraphDistance::PathPointPair > oldPath )
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
	return otherGraph->getEdge(link->property["correspond"].toInt());
}

void Task::setNode( QString node_ID )
{
	property["nodeID"] = node_ID;
	this->nodeID = node_ID;

	node()->property["task"].setValue( this );
	node()->property["taskType"] = type;
	node()->property["taskIsDone"] = false;
	node()->property["taskIsReady"] = false;

	// Override default colors for split and merge
	QString snID = node()->id, tnID = targetNode()->id;

	node()->property["taskTypeReal"] = type;

	if( snID.contains("_") && !snID.contains("_null") ) 
	{
		mycolor = TaskColors[Task::SPLIT];
		node()->property["taskTypeReal"] = Task::SPLIT;
	}
	if( tnID.contains("_") && !tnID.contains("_null") ) 
	{
		mycolor = TaskColors[Task::MERGE];
		node()->property["taskTypeReal"] = Task::MERGE;
	}

	targetNode()->property["taskTypeReal"] = node()->property["taskTypeReal"];
}

std::vector<RMF::Frame> Task::smoothRotateFrame( RMF::Frame sframe, Eigen::Quaterniond & rotation, int steps )
{
	std::vector<RMF::Frame> frames;

	double stepSize = 1.0 / double(steps);

	Eigen::Quaterniond eye = Eigen::Quaterniond::Identity();

	for(double alpha = 0; alpha <= 1.0; alpha += stepSize)
	{
		Eigen::Vector3d r = sframe.r, s = sframe.s, t = sframe.t;
		r = eye.slerp(alpha, rotation) * r;
		s = eye.slerp(alpha, rotation) * s;
		t = eye.slerp(alpha, rotation) * t;

		RMF::Frame curFrame = RMF::Frame((r),(s),(t));
		curFrame.center = sframe.center;
		frames.push_back(curFrame);
	}
	return frames;
}

void Task::prepareMorphEdges()
{
	Node * n = node(), *tn = targetNode();
	QVector<Link*> allEdges = active->getEdges(n->id), edges;

	// Filter edges
	foreach(Link * l, allEdges)
	{
		Structure::Node* other = l->otherNode(n->id);

		// Skip shrunk and non-grown nodes
		if (other->property["shrunk"].toBool()) continue;
		if (other->property["taskType"].toInt() == Task::GROW && !other->property["taskIsDone"].toBool()) continue;
		
		// Skip edges that will leave me in future
		Structure::Link* tl = target->getEdge(l->property["correspond"].toInt());
		if (!tl->hasNode(tn->id)) continue;

		edges.push_back(l);
	}

	property["edges"].setValue( edges );

	// Compute paths for all edges
	foreach(Link * link, edges)
	{
		Vector3d start = link->positionOther(n->id);
		NodeCoord futureNodeCord = futureOtherNodeCoord(link);
		Vector3d end = active->position(futureNodeCord.first, futureNodeCord.second);

		// Geodesic distances on the active graph excluding the running tasks
		QVector< GraphDistance::PathPointPair > path;
		
		QString otherOld = link->otherNode(n->id)->id;
		QString otherNew = futureNodeCord.first;

		if( otherOld == otherNew )
		{
			GraphDistance gd( active->getNode(otherOld) );
			gd.computeDistances( end, DIST_RESOLUTION );	
			gd.smoothPathCoordTo( start, path );
		}
		else
		{
			QVector<QString> exclude = active->property["activeTasks"].value< QVector<QString> >();
			GraphDistance gd( active, exclude );
			gd.computeDistances( end, DIST_RESOLUTION );	
			gd.smoothPathCoordTo( start, path );
		}

		// Check
		if(path.isEmpty()) path.push_back(GraphDistance::PathPointPair( PathPoint(futureNodeCord.first, futureNodeCord.second)));

		// End of path should be exactly as in target
		path.back() = GraphDistance::PathPointPair( PathPoint(futureNodeCord.first, futureNodeCord.second)  );

		link->setProperty("path", path);
	}
}

void Task::executeMorphEdges( double t )
{
	Node * n = node();

	QVector<Structure::Link*> edges = property["edges"].value< QVector<Structure::Link*> >();
	foreach (Structure::Link* link, edges)
	{
		QVector< GraphDistance::PathPointPair > path = link->property["path"].value< QVector< GraphDistance::PathPointPair > >();
		if(!path.size()) continue;

		// Local link coordinates
		int idx = t * (path.size() - 1);
		GraphDistance::PathPointPair cur = path[idx];
		Structure::Node *otherOld = link->otherNode(n->id);
		Structure::Node *otherNew = active->getNode(cur.a.first);

		// Do not perform on active nodes
		Task * otherTask = otherOld->property["task"].value<Task*>();
		if(otherTask->isReady && !otherTask->isDone) continue;

		link->replace( otherOld->id, otherNew, Array1D_Vector4d(1,cur.a.second) );
	}
}

bool Task::isCrossing()
{
	Structure::Node *n = node(), *tn = targetNode();

	bool isCross = false;

	if( isReady && !isDone )
	{
		foreach(Link * l, active->getEdges(n->id))
		{
			if(l->property["modified"].toBool())
				continue;

			// check if my neighbor will change
			Structure::Link* tl = target->getEdge(l->property["correspond"].toInt());
			Structure::Node* sNb = l->otherNode(n->id);
			QString tNbID = tl->otherNode(tn->id)->id;
			if (sNb->property["correspond"].toString() != tNbID){
				isCross = true;
				break;
			}
		}
	}

	property["isCrossing"] = isCross;
	return isCross;
}

bool Task::isCutting()
{
	bool result = false;

	{
		Structure::Graph copyActive (*active);

		// Exclude nodes that is active and non-existing
		QSet<QString> excludeNodes;

		foreach(QString nid, active->property["activeTasks"].value< QVector<QString> >())
			excludeNodes.insert(nid);

		// Keep myself in graph for checking
		excludeNodes.remove(nodeID);
		foreach (QString nid, excludeNodes)	copyActive.removeNode(nid);

		result = copyActive.isCutNode(nodeID);
	}

	return result;
}

bool Task::isCuttingReal()
{
	Structure::Graph * copyActive = new Structure::Graph(*active);

	// Exclude nodes that is active and non-existing
	QSet<QString>excludeNodes;

	foreach(QString nid, active->property["activeTasks"].value< QVector<QString> >())
		excludeNodes.insert(nid);

	// Skip un-grown nodes
	foreach(Node* n, active->nodes)
		if (ungrownNode(n->id)) excludeNodes.insert(n->id);

	// Keep myself in graph for checking
	excludeNodes.remove(nodeID);
	foreach (QString nid, excludeNodes)	copyActive->removeNode(nid);

	bool result = copyActive->isCutNode(nodeID);
	delete copyActive;
	return result;
}

bool Task::ungrownNode( QString nid )
{
	Task* t = active->getNode(nid)->property["task"].value<Task*>();
	return t->type == GROW && !t->isReady;
}

QVector<Task*> Task::allOtherTasks()
{
	QVector<Task*> result;
	foreach(Node * n, active->nodes){
		if(n->id != nodeID){
			result.push_back( n->property["task"].value<Task*>() );
		}
	}
	return result;
}
