#include "Scheduler.h"
#include "Task.h"
#include "ARAPCurveDeformer.h"
#include <QGraphicsSceneMouseEvent>

#include "Synthesizer.h"

#include "weld.h"

typedef std::vector< std::pair<double,double> > VectorPairDouble;
Q_DECLARE_METATYPE(Vector3);
Q_DECLARE_METATYPE(Vec4d);
Q_DECLARE_METATYPE(VectorPairDouble);
Q_DECLARE_METATYPE(ARAPCurveDeformer*);

int globalCount = 0;

Task::Task( Structure::Graph * activeGraph, Structure::Graph * targetGraph, TaskType taskType, int ID )
{
	// Task properties
	this->active = activeGraph;
	this->target = targetGraph;

	this->type = taskType;
	this->taskID = ID;
	this->mycolor = TaskColors[taskType];
	this->start = 0;
	this->length = 80;
	this->currentTime = start;
	this->isReady = false;
	this->isDone = false;

	this->arapIterations = 3;

	// Visual properties
	width = length;
	height = 17;
	setFlags(ItemIsMovable | ItemIsSelectable | ItemSendsGeometryChanges);
	isResizing = false;
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

QVector<Structure::Link *> Task::getGoodEdges()
{
	QVector<Structure::Link *> goodEdges;

	foreach(Structure::Link * edge, active->getEdges(node()->id))
	{
		if( edge->property.contains("isCut") ) continue;

		goodEdges.push_back(edge);
	}

	return goodEdges;
}

QList<Structure::Link*> Task::furthermostGoodEdges()
{
	QList<Structure::Link*> allEdges = active->furthermostEdges(node()->id);
	
	foreach(Structure::Link * edge, allEdges)
	{
		if( edge->property.contains("isCut") )
			allEdges.removeAll(edge);
	}

	return allEdges;
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

void Task::geometryMorph( double t )
{
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

void Task::deformCurve( int anchorPoint, int controlPoint, Vec3d newControlPos )
{
	Structure::Curve * curve = (Structure::Curve*)node();

	ARAPCurveDeformer * deformer = NULL;

	if(!property.contains("deformer"))
		property["deformer"].setValue( deformer = new ARAPCurveDeformer( curve->curve.mCtrlPoint, curve->curve.mCtrlPoint.size() * 0.5 ) );
	else
		deformer = property["deformer"].value<ARAPCurveDeformer*>();

	// From current state
	deformer->points = curve->curve.mCtrlPoint;

	deformer->ClearAll();
	deformer->setControl(controlPoint);
	deformer->SetAnchor(anchorPoint);
	deformer->MakeReady();

	// Update and deform
	deformer->UpdateControl(controlPoint, newControlPos);
	deformer->Deform( arapIterations );

	// Update curve geometry
	curve->setControlPoints( deformer->points );
}

void Task::weldMorphPath()
{
	QVector< NodeCoord > cleanPath;
	QVector< NodeCoord > oldPath = property["path"].value< QVector< NodeCoord > >();
	
	// Find spatial position
	std::vector<Vec3d> spatialPath;
	QSet<int> allPath;
	foreach(NodeCoord nc, oldPath) 
	{
		spatialPath.push_back( active->position(nc.first, nc.second) );
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
	property["path"].setValue( cleanPath );
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

Vec3d Task::futureLinkPosition( Structure::Link *link )
{
	NodeCoord fnc = futureOtherNodeCoord(link);
	return active->getNode(fnc.first)->position(fnc.second);
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

// PREPARE
void Task::prepare()
{
	this->start = this->x();
	this->currentTime = start;
	this->isDone = false;

	this->property["orgCtrlPoints"].setValue( node()->controlPoints() );

	QVector<QString> runningTasks = active->property["running_tasks"].value< QVector<QString> >();
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
			prepareGrowSheet();
			break;
		case SHRINK:
			prepareShrinkSheet();
			break;
		case SPLIT:
		case MERGE:
		case MORPH:
			break;
		}
	}

	this->isReady = true;
	node()->property["isReady"] = true;
}

void Task::prepareShrinkCurve()
{
	Structure::Node * n = node();
	QVector<Structure::Link*> edges = active->getEdges(n->id);

	if( active->isCutNode(n->id) )
	{
		active->printLinksInfo();

		property["isCutNode"] = true;
		prepareShrinkCurveConstrained();
		return;
	}

	
	if(edges.size() == 1)
	{
		Structure::Link * l = edges.front();
		Structure::Node * base = l->otherNode(n->id);

		Structure::Curve* structure_curve = ((Structure::Curve*)n);

		Vec4d coordBase = l->getCoord(base->id).front();
		Vec4d coordSelf = l->getCoord(n->id).front();

		Vector3 linkPositionBase = l->position( base->id );
		int cpIDX = structure_curve->controlPointIndexFromCoord( coordSelf );

		// Curve folding
		Array1D_Vector3 deltas = structure_curve->foldTo( coordSelf, false );
		deltas = inverseVectors3(deltas);

		// Growing / shrinking instructions
		property["deltas"].setValue( deltas );
		property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );
	}

		
	if(edges.size() == 2)
	{
		// Links and positions (on myself)
		Structure::Link * linkA = edges.front();
		Structure::Link * linkB = edges.back();
		Vec3d pointA = linkA->position( n->id );
		Vec3d pointB = linkB->position( n->id );

		// Geodesic distance between two link positions on the active graph excluding the running tasks
		QVector< NodeCoord > path;
		QVector<QString> exclude = active->property["running_tasks"].value< QVector<QString> >();
		GraphDistance gd( active, exclude );
		gd.computeDistances( pointA );
		gd.pathCoordTo( pointB, path);

		// Use the center of the path as the end point
		NodeCoord endPointCoord = path[path.size() / 2];
		Vec3d endPoint = active->position(endPointCoord.first, endPointCoord.second);

		// Separate the path into two for linkA and linkB
		int N = path.size(), hN = N / 2;
		if (N % 2 == 0) {
			path.insert(hN, path[hN]);
			N++;
		}

		QVector<NodeCoord> pathA, pathB;
		for (int i = 0; i < N/2; i++)
		{
			pathA.push_back(path[i]);
			pathB.push_back(path[N-1-i]);
		}

		property["pathA"].setValue(pathA);
		property["pathB"].setValue(pathB);

		// ARAP curve deformation
		Structure::Curve* structure_curve = ((Structure::Curve*)n);

		int cpidxA = structure_curve->controlPointIndexFromCoord(linkA->getCoord(n->id).front());
		int cpidxB = structure_curve->controlPointIndexFromCoord(linkB->getCoord(n->id).front());
		property["cpidxA"].setValue(cpidxA);
		property["cpidxB"].setValue(cpidxB);

		property["deformer"].setValue( new ARAPCurveDeformer( structure_curve->curve.mCtrlPoint ) );
	}
}

void Task::prepareGrowCurve()
{
	Structure::Node * n = node();
	Structure::Node * tn = targetNode();
	QVector<Structure::Link*> edges = active->getEdges(n->id);
	QVector<Structure::Link*> tedges = target->getEdges(tn->id);

	Structure::Curve* structure_curve = ((Structure::Curve*)n);
	Structure::Curve* t_structure_curve = ((Structure::Curve*)tn);

	if( target->isCutNode(tn->id) )
	{
		property["isCutNode"] = true;
		prepareGrowCurveConstrained();
		return;
	}

	if (tedges.size() == 1)
	{
		Structure::Link * tl = tedges.front();
		Structure::Node * tbase = tl->otherNode(tn->id);

		Vec4d coordBase = tl->getCoord(tbase->id).front();
		Vec4d coordSelf = tl->getCoord(tn->id).front();

		QString baseID = tbase->property["correspond"].toString();
		Structure::Node* base = active->getNode(baseID);

		Vector3 linkPositionBase = base->position(coordBase);
		int cpIDX = structure_curve->controlPointIndexFromCoord( coordSelf );

		// Place curve
		structure_curve->moveBy( linkPositionBase - structure_curve->controlPoint(cpIDX)  );

		// Curve folding
		Array1D_Vector3 deltas = structure_curve->foldTo( coordSelf, true );

		// Growing instructions
		property["deltas"].setValue( deltas );
		property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );
	}

	if (tedges.size() == 2)
	{
		// Links, nodes and positions on the target
		Structure::Link *tlinkA = tedges.front();
		Structure::Link *tlinkB = tedges.back();
		Structure::Node *totherA = tlinkA->otherNode(tn->id);
		Structure::Node *totherB = tlinkB->otherNode(tn->id);
		Vec4d othercoordA = tlinkA->getCoord(totherA->id).front();
		Vec4d othercoordB = tlinkB->getCoord(totherB->id).front();

		// Corresponding stuff on active
		QString otherAID = totherA->property["correspond"].toString();
		QString otherBID = totherB->property["correspond"].toString();
		Structure::Node *otherA = active->getNode(otherAID);
		Structure::Node *otherB = active->getNode(otherBID);
		Vec3d pointA = otherA->position(othercoordA);
		Vec3d pointB = otherB->position(othercoordB);

		// Geodesic distance between two link positions on the active graph excluding the running tasks
		QVector< NodeCoord > path;
		QVector<QString> exclude = active->property["running_tasks"].value< QVector<QString> >();
		GraphDistance gd( active, exclude );
		gd.computeDistances( pointA );
		gd.pathCoordTo( pointB, path);

		// Separate the path into two for linkA and linkB
		int N = path.size(), hN = N / 2;
		if (N %2 == 0) path.insert(hN, path[hN]);

		QVector<NodeCoord> pathA, pathB;
		for (int i = 0; i < hN; i++)
		{
			pathA.push_back(path[hN-1-i]);
			pathB.push_back(path[hN+1+i]);
		}


		property["pathA"].setValue(pathA);
		property["pathB"].setValue(pathB);

		// Use the center of the path as the start point
		NodeCoord startPointCoord = path[path.size() / 2];
		Vec3d startPoint = active->position(startPointCoord.first, startPointCoord.second);

		// ARAP curve deformation
		int cpidxA = t_structure_curve->controlPointIndexFromCoord( tlinkA->getCoord(tn->id).front() );
		int cpidxB = t_structure_curve->controlPointIndexFromCoord( tlinkB->getCoord(tn->id).front() );
		property["cpidxA"].setValue(cpidxA);
		property["cpidxB"].setValue(cpidxB);

		std::vector<Vec3d> originalPoints = structure_curve->curve.mCtrlPoint;
		ARAPCurveDeformer * deformer = new ARAPCurveDeformer( originalPoints );
		property["deformer"].setValue( deformer );

		// Initial position of n
		structure_curve->foldTo(Vec4d(0.5), true);
		structure_curve->curve.translate(startPoint - structure_curve->position(Vec4d(0.5)));	

		deformer->points = structure_curve->curve.mCtrlPoint;
	}
}

void Task::prepareShrinkCurveConstrained()
{
	Structure::Node * n = node();
	Structure::Curve* structure_curve = ((Structure::Curve*)n);

	if (property.contains("isCutNode"))
	{
		// Find first link to a sheet
		QVector<Structure::Link*> my_edges = active->getEdges(n->id); 

		Structure::Link * baseLink = my_edges.front();
		foreach(Structure::Link * edge, my_edges){
			Structure::Node *othernode = edge->otherNode(n->id);
			if(othernode->type() == Structure::SHEET){
				baseLink = edge;
				break;
			}
		}

		Vec4d coordSelf = baseLink->getCoord(n->id).front();
		Structure::Node *basenode = baseLink->otherNode(n->id);

		// Curve folding
		Array1D_Vector3 deltas = structure_curve->foldTo( coordSelf, false );
		deltas = inverseVectors3(deltas);

		property["deltas"].setValue( deltas );
		property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );
		property["anchorNode"].setValue( basenode->id );
	}
}

void Task::prepareGrowCurveConstrained()
{
	Structure::Node *n = node();
	Structure::Node *tn = targetNode();
	Structure::Curve* structure_curve = ((Structure::Curve*)n);

	if (property.contains("isCutNode"))
	{
		// Find first link to a sheet
		QVector<Structure::Link*> t_my_edges = target->getEdges(tn->id); 

		Structure::Link * tbaseLink = t_my_edges.front();
		foreach(Structure::Link * tedge, t_my_edges){
			Structure::Node *tothernode = tedge->otherNode(tn->id);
			if(tothernode->type() == Structure::SHEET){
				tbaseLink = tedge;
				break;
			}
		}

		Vec4d coordSelf = tbaseLink->getCoord(tn->id).front();
		Structure::Node *tbasenode = tbaseLink->otherNode(tn->id);
		QString basenodeID = tbasenode->property["correspond"].toString();
		
		// Curve folding
		Array1D_Vector3 deltas = structure_curve->foldTo( coordSelf, true );

		property["deltas"].setValue( deltas );
		property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );
		property["anchorNode"].setValue( basenodeID );
	}
}

void Task::prepareMorphCurve()
{
	Structure::Node * n = node();
	Structure::Curve* structure_curve = ((Structure::Curve*)node());
	Structure::Node * tn = targetNode();
	QVector<Structure::Link*> edges = active->getEdges(n->id);

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		Structure::Link * link = edges.front();
		Vec3d startPoint = link->position(node()->id);

		// Compute path
		QVector<QString> exclude = active->property["running_tasks"].value< QVector<QString> >();
		GraphDistance graphDist(active, exclude);
		QVector< NodeCoord > path;
		graphDist.computeDistances( link->positionOther( node()->id ) );
		graphDist.pathCoordTo( startPoint, path);

		property["path"].setValue( path );
		property["cpIDX"] = structure_curve->controlPointIndexFromCoord( link->getCoord(node()->id).front() );
	}

	// 2) TWO edges
	if(edges.size() == 2)
	{
		// Start and end for both links
		Structure::Link * linkA = edges.front();
		Vec3d startA = linkA->position(n->id);
		Vec3d endA = futureLinkPosition(linkA);

		Structure::Link * linkB = edges.back();
		Vec3d startB = linkB->position(n->id);
		Vec3d endB = futureLinkPosition(linkB);

		// Geodesic distances on the active graph excluding the running tasks
		QVector< NodeCoord > pathA, pathB;	
		QVector<QString> exclude = active->property["running_tasks"].value< QVector<QString> >();
		GraphDistance gd( active, exclude );

		gd.computeDistances( endA );	gd.pathCoordTo( startA, pathA);
		gd.computeDistances( endB );	gd.pathCoordTo( startB, pathB);

		property["pathA"].setValue(pathA);
		property["pathB"].setValue(pathB);

		// To-do
		// Swap the correspondence of two links

		// ARAP curve deformation
		int cpidxA = structure_curve->controlPointIndexFromCoord(linkA->getCoord(n->id).front());
		int cpidxB = structure_curve->controlPointIndexFromCoord(linkB->getCoord(n->id).front());
		property["cpidxA"].setValue(cpidxA);
		property["cpidxB"].setValue(cpidxB);

		property["deformer"].setValue( new ARAPCurveDeformer( structure_curve->curve.mCtrlPoint ) );
	}
}

void Task::prepareShrinkSheet()
{
	Structure::Node * n = node();
	QVector<Structure::Link*> edges = active->getEdges(n->id);

	if (edges.size() == 1)
	{
		Structure::Link * l = edges.front();
		Structure::Node * base = l->otherNode(n->id);
		Structure::Sheet* structure_sheet = ((Structure::Sheet*)n);

		// Placement:
		structure_sheet->moveBy( l->position( base->id ) - l->position( n->id ) );

		// Sheet folding:
		Array2D_Vector3 deltas = structure_sheet->foldTo( l->getCoord(n->id), false );
		deltas = inverseVectors3(deltas);

		// Shrinking instructions
		property["deltas"].setValue( deltas );
		property["orgCtrlPoints"].setValue( structure_sheet->surface.mCtrlPoint );
	}
}

void Task::prepareGrowSheet()
{
	Structure::Node * n = node();
	Structure::Node * tn = targetNode();
	QVector<Structure::Link*> edges = active->getEdges(n->id);
	QVector<Structure::Link*> tedges = target->getEdges(tn->id);
	Structure::Sheet* structure_sheet = ((Structure::Sheet*)n);

	if (tedges.size() == 1)
	{
		Structure::Link * tl = tedges.front();
		Structure::Node * tbase = tl->otherNode(tn->id);

		QString baseID = tbase->property["correspond"].toString();
		Structure::Node * base = active->getNode(baseID);

		// Placement:
		//structure_sheet->moveBy( l->position( base->id ) - l->position( n->id ) );

		// Sheet folding:
		Array2D_Vector3 deltas = structure_sheet->foldTo( tl->getCoord(tn->id), true );

		// Growing / shrinking instructions
		property["deltas"].setValue( deltas );
		property["orgCtrlPoints"].setValue( structure_sheet->surface.mCtrlPoint );
	}
}

void Task::prepareMorphSheet()
{

}


// EXECUTE
void Task::execute( double t )
{	
	if(!isActive(t)) return;

	if ( !isReady ) prepare();

	currentTime = start + (t * length);

	// Blend geometries
	geometryMorph( t );

	if (node()->type() == Structure::CURVE)
	{
		switch(type)
		{
		case GROW:
			executeGrowCurve(t);
			break;
		case SHRINK:
			executeShrinkCurve( t );
			break;
		case MORPH:
			executeMorphCurve(t);
			break;
		}
	}
	else
	{
		switch(type)
		{
		case GROW:
		case SHRINK:
			executeGrowShrinkSheet( t );
			break;
		case MORPH:
			executeMorphSheet(t);
			break;
		}
	}



	if(t == 1.0)
	{
		this->isDone = true;
	}
}

void Task::executeShrinkCurve( double t )
{
	Structure::Node *n = node();
	QVector<Structure::Link*> edges = active->getEdges(n->id);


	// Cut node
	if (property.contains("isCutNode"))
	{
		executeCurveConstrained(t);
		n->property["isReady"] = false;
		return;
	}

	// Regular shrink
	if (edges.size() == 1)
		foldCurve(t);
	else if (edges.size() == 2)
		executeMorphCurve(t);

	
	// When task is done
	if (t == 1)
	{
		n->property["isReady"] = false;

		// Delete all edges
		foreach(Structure::Link *link, edges)
			active->removeEdge(link->n1, link->n2);
	}
}

void Task::executeGrowCurve( double t )
{
	Structure::Node * tn = targetNode();
	QVector<Structure::Link*> tedges = target->getEdges(tn->id);

	// Cut node
	if (property.contains("isCutNode"))
	{
		executeCurveConstrained(t);
		return;
	}

	// Regular grow
	if (tedges.size() == 1)
		foldCurve(t);
	
	if (tedges.size() == 2)
		executeMorphCurve(t);
}

void Task::foldCurve( double t )
{
	Structure::Curve* structure_curve = ((Structure::Curve*)node());

	Array1D_Vector3 cpts = property["orgCtrlPoints"].value<Array1D_Vector3>();
	Array1D_Vector3 deltas = property["deltas"].value<Array1D_Vector3>();

	// Grow curve
	for(int u = 0; u < structure_curve->curve.mNumCtrlPoints; u++)
		structure_curve->curve.mCtrlPoint[u] = cpts[u] + (deltas[u] * t);	
}

void Task::executeCurveConstrained( double t )
{
	Structure::Node * n = node();
	Structure::Curve* current_curve = ((Structure::Curve*)n);

	// Grow / shrink the node
	Array1D_Vector3 cpts = property["orgCtrlPoints"].value<Array1D_Vector3>();
	Array1D_Vector3 deltas = property["deltas"].value<Array1D_Vector3>();
	QString anchorNode = property["anchorNode"].toString();

	// Grow / shrink curve
	for(int u = 0; u < current_curve->curve.mNumCtrlPoints; u++)
		current_curve->curve.mCtrlPoint[u] = cpts[u] + (deltas[u] * t);

	// Re-link:
	foreach( Structure::Link * edge, active->getEdges(n->id) )
	{
		Structure::Node * otherNode = edge->otherNode(n->id);
		if(otherNode->id == anchorNode) continue;

		Structure::Curve* other_curve = ((Structure::Curve*) otherNode);

		int nCtrl = other_curve->curve.GetNumCtrlPoints();
		int idx_control = other_curve->controlPointIndexFromCoord( edge->getCoord(other_curve->id).front() );
		int idx_anchor = (idx_control > nCtrl / 2) ? 0 : nCtrl - 1;

		ARAPCurveDeformer * deformer = new ARAPCurveDeformer( other_curve->controlPoints(), other_curve->controlPoints().size() * 0.5 );

		deformer->ClearAll();
		deformer->setControl( idx_control );
		deformer->SetAnchor( idx_anchor );
		deformer->MakeReady();

		Vec3d newPosCtrl = edge->position(n->id);
		deformer->UpdateControl(idx_control, newPosCtrl);
		deformer->Deform( arapIterations );
		other_curve->setControlPoints( deformer->points );
	}
}

void Task::executeGrowShrinkSheet( double t )
{
	Structure::Node * n = node();
	Structure::Node * tn = targetNode();
	QVector<Structure::Link*> edges = active->getEdges(n->id);
	QVector<Structure::Link*> tedges = target->getEdges(tn->id);


	if ((type == SHRINK && edges.size() == 1)
		|| (type == GROW && tedges.size() == 1))
	{
		Structure::Sheet* structure_sheet = ((Structure::Sheet*)node());

		Array2D_Vector3 cpts = property["orgCtrlPoints"].value<Array2D_Vector3>();
		Array2D_Vector3 deltas = property["deltas"].value<Array2D_Vector3>();

		// Grow sheet
		for(int u = 0; u < structure_sheet->surface.mNumUCtrlPoints; u++)
			for(int v = 0; v < structure_sheet->surface.mNumVCtrlPoints; v++)
				structure_sheet->surface.mCtrlPoint[u][v] = cpts[u][v] + (deltas[u][v] * t);
	}

	// When the task is done
	if ( t == 1)
	{
		if (type == SHRINK)
		{
			n->property["isReady"] = false;

			// Delete all edges
			foreach(Structure::Link *link, edges)
				active->removeEdge(link->n1, link->n2);
		}

		if (type == GROW)
		{
			copyTargetEdge(tedges.front());
		}
	}
}

void Task::executeMorphCurve( double t )
{
	Structure::Node * n = node();
	QVector<Structure::Link*> edges = active->getEdges(n->id);			
	Structure::Node * tn = targetNode();
	QVector<Structure::Link*> tedges = target->getEdges(tn->id);

	Structure::Curve* structure_curve = ((Structure::Curve*)n);

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		Structure::Link * edge = edges.front();
		Structure::Curve* current_curve = ((Structure::Curve*)n);
		Structure::Curve* target_curve = targetCurve();

		weldMorphPath();
		QVector< NodeCoord > path = property["path"].value< QVector< NodeCoord > >();

		int cpIDX = property["cpIDX"].toInt();

		if(path.size() > 0) 
		{
			int current = t * (path.size() - 1);
			Vec3d newPos = active->position(path[current].first,path[current].second);

			// Move end with a link
			current_curve->curve.translateTo( newPos, cpIDX );

			QString tbaseNode = edge->otherNode(current_curve->id)->property["correspond"].toString();
			QString tnodeID = current_curve->property["correspond"].toString();

			Structure::Link * tbaseNode_tnode = target->getEdge(tbaseNode, tnodeID);

			int cpIDX_target = target_curve->controlPointIndexFromCoord(tbaseNode_tnode->getCoord(tnodeID).front());

			target_curve->curve.translateTo( newPos, cpIDX_target); // moves along for the ride..
		}

		// Move free end (linear interpolation)
		int freeEnd = (cpIDX < current_curve->curve.GetNumCtrlPoints() * 0.5) ? 
			current_curve->curve.GetNumCtrlPoints() - 1 : 0;

		Vec3d freeEndPos = AlphaBlend(pow(t,2), current_curve->controlPoint(freeEnd), target_curve->controlPoint(freeEnd));
		deformCurve(cpIDX, freeEnd, freeEndPos);
	}

	// 2) TWO edges
	if (edges.size() == 2 || type == Task::GROW )
	{
		int cpidxA = property["cpidxA"].toInt();
		int cpidxB = property["cpidxB"].toInt();

		ARAPCurveDeformer * deformer = property["deformer"].value<ARAPCurveDeformer*>();

		QVector< NodeCoord > pathA = property["pathA"].value< QVector< NodeCoord > >();
		QVector< NodeCoord > pathB = property["pathB"].value< QVector< NodeCoord > >();

		int idxA = t * (pathA.size() - 1);
		int idxB = t * (pathB.size() - 1);

		// Move point A to next step
		if(idxA > 0 && pathA.size() > 2)
		{
			NodeCoord stepA = pathA[idxA];
			deformer->ClearAll();
			deformer->setControl(cpidxA);
			deformer->SetAnchor(cpidxB);
			deformer->MakeReady();

			Vec3d newPosA = active->position(stepA.first, stepA.second);
			deformer->UpdateControl(cpidxA, newPosA);
			deformer->Deform( arapIterations );
			structure_curve->setControlPoints( deformer->points );
		}

		// Move point B to next step
		if(idxB > 0 && pathB.size() > 2)
		{
			NodeCoord stepB = pathB[idxB];
			deformer->ClearAll();
			deformer->setControl(cpidxB);
			deformer->SetAnchor(cpidxA);
			deformer->MakeReady();

			Vec3d newPosB = active->position(stepB.first, stepB.second);
			deformer->UpdateControl(cpidxB, newPosB);
			deformer->Deform( arapIterations );
			structure_curve->setControlPoints( deformer->points );
		}
	}


	// When this task is done
	if (t == 1)
	{

		// There are two edges in the active should be killed
		if (type == Task::SHRINK)
		{
			if (edges.size() == 2)
			{
				Structure::Link *linkA = edges.front();
				Structure::Link *linkB = edges.back();

				active->removeEdge(linkA->n1, linkA->n2);
				active->removeEdge(linkB->n1, linkB->n2);
			}
		}

		// There are no edge in active but two in the target
		if (type == Task::GROW)
		{
			copyTargetEdge(tedges.front());
			copyTargetEdge(tedges.back());
		}

		// There are one or two edges need to be updated
		if (type == Task::MORPH)
		{
			foreach(Structure::Link *link, edges)
			{
				QString otherNode = link->otherNode(n->id)->id;

				NodeCoord fnc = futureOtherNodeCoord(link);

				link->replace( otherNode, active->getNode(fnc.first), std::vector<Vec4d>(1,fnc.second) );
			}
		}

	}
}

void Task::executeMorphSheet( double t )
{
	
}
