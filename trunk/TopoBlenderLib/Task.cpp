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
	this->length = 120;
	this->currentTime = start;
	this->isReady = false;
	this->isDone = false;

	this->arapIterations = 4;

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
	painter->drawText(boundingRect(), QString("%1 - Len: %2").arg(TaskNames[type]).arg(width), QTextOption(Qt::AlignVCenter | Qt::AlignHCenter));
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

void Task::prepare()
{
	this->start = this->x();
	this->currentTime = start;
	this->isDone = false;

	this->property["orgCtrlPoints"].setValue( node()->controlPoints() );

	switch(type)
	{
	case GROW:
	case SHRINK:
		prepareGrowShrink();
		break;
	case SPLIT:
	case MERGE:
	case MORPH:
		prepareMorph();
		break;
	}

	this->isReady = true;
}

void Task::execute( double t )
{	
	if(t < 0.0 || !isReady || isDone) return;

	currentTime = start + (t * length);

	// Blend geometries
	geometryMorph( t );

	switch(type)
	{
	case GROW:
	case SHRINK:
		executeGrowShrink( t );
		break;
	case SPLIT:
	case MERGE:
	case MORPH:
		executeMorph( t );
		break;
	}

	if(t == 1.0)
	{
		this->isDone = true;
	}
}

void Task::prepareGrowShrink()
{
	Structure::Node * n = node();
	QVector<Structure::Link*> edges = getGoodEdges();

	if( property.contains("isCutNode") )
	{
		if(n->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)n);

			Vec3d pointA = structure_curve->position( Vec4d(0) );
			Vec3d pointB = structure_curve->position( Vec4d(1) );

			// Find first link to a sheet
			QVector<Structure::Link*> my_edges = active->getEdges(n->id); 

			if(my_edges.size())
			{
				Structure::Link * endLink = my_edges.front();
				foreach(Structure::Link * edge, my_edges){
					if(edge->otherNode(n->id)->type() == Structure::SHEET){
						endLink = edge;
						break;
					}
				}

				// Curve folding
				bool isApplyFold = (type == GROW) ? true : false;
				Array1D_Vector3 deltas = structure_curve->foldTo( endLink->getCoord(n->id).front(), isApplyFold );
				if(!isApplyFold) deltas = inverseVectors3(deltas);

				property["deltas"].setValue( deltas );
				property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );
				property["anchorNode"].setValue( endLink->otherNode(n->id)->id );
			}
		}

		if(n->type() == Structure::SHEET)
		{
		}

		return;
	}

	// Special case?
	if(edges.size() == 0){
		if(type == SHRINK){
			Structure::Link * singleEdge = active->getEdges(n->id).front();
			edges.push_back(singleEdge);
		}
	}

	if(edges.size() == 1)
	{
		Structure::Link * l = edges.front();
		Structure::Node * base = l->otherNode(n->id);

		if(n->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)n);

			Vec4d coordBase = l->getCoord(base->id).front();
			Vec4d coordSelf = l->getCoord(n->id).front();

			Vector3 linkPositionBase = l->position( base->id );
			int cpIDX = structure_curve->controlPointIndexFromCoord( coordSelf );

			// Place curve
			structure_curve->moveBy( linkPositionBase - structure_curve->controlPoint(cpIDX)  );

			// Curve folding
			bool isApplyFold = (type == GROW) ? true : false;
			Array1D_Vector3 deltas = structure_curve->foldTo( coordSelf, isApplyFold );
			if(!isApplyFold) deltas = inverseVectors3(deltas);

			// Growing / shrinking instructions
			property["deltas"].setValue( deltas );
			property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );
		}

		if(n->type() == Structure::SHEET)
		{
			Structure::Link * l = edges.front();
			Structure::Node * base = l->otherNode(n->id);
			Structure::Sheet* structure_sheet = ((Structure::Sheet*)n);

			// Placement:
			structure_sheet->moveBy( l->position( base->id ) - l->position( n->id ) );

			// Sheet folding:
			bool isApplyFold = (type == GROW) ? true : false;
			Array2D_Vector3 deltas = structure_sheet->foldTo( l->getCoord(n->id), isApplyFold );
			if(!isApplyFold) deltas = inverseVectors3(deltas);

			// Growing / shrinking instructions
			property["deltas"].setValue( deltas );
			property["orgCtrlPoints"].setValue( structure_sheet->surface.mCtrlPoint );
		}
	}

	if(edges.size() == 2)
	{
		if(node()->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)n);

			property["orgCtrlPoints"].setValue( structure_curve->curve.mCtrlPoint );

			QList<Structure::Link*> farEdges = furthermostGoodEdges();

			Structure::Link * linkA = farEdges.front();
			Structure::Link * linkB = farEdges.back();

			Vec3d pointA = linkA->position( n->id );
			Vec3d pointB = linkB->position( n->id );

			QVector< NodeCoord > path;

			GraphDistance gd( active, SingleNode(node()->id) );
			gd.computeDistances( pointA );
			gd.pathCoordTo( pointB, path);

			NodeCoord endPointCoord = path[path.size() / 2];
			Vec3d endPoint = active->position(endPointCoord.first, endPointCoord.second);

			if(type == GROW)
			{
                Vec4d halfCoord(0.5);
                structure_curve->foldTo( halfCoord, true );
				structure_curve->moveBy( endPoint - structure_curve->position(Vec4d(0.5)) );
			}
				
			if(type == SHRINK)
			{
				linkA->replace( linkA->otherNode(n->id)->id, active->getNode(endPointCoord.first), Array1D_Vec4d(1, endPointCoord.second));
				linkB->replace( linkB->otherNode(n->id)->id, active->getNode(endPointCoord.first), Array1D_Vec4d(1, endPointCoord.second));
			}

			// Morph operation need these
			linkA->property["finalCoord_n1"].setValue( qMakePair(linkA->n1->id, linkA->getCoord(linkA->n1->id)) );
			linkA->property["finalCoord_n2"].setValue( qMakePair(linkA->n2->id, linkA->getCoord(linkA->n2->id)) );

			linkB->property["finalCoord_n1"].setValue( qMakePair(linkB->n1->id, linkB->getCoord(linkB->n1->id)) );
			linkB->property["finalCoord_n2"].setValue( qMakePair(linkB->n2->id, linkB->getCoord(linkB->n2->id)) );

			// ARAP curve deformation
			setupCurveDeformer( structure_curve, linkA, linkB );
			ARAPCurveDeformer* deformer = property["deformer"].value<ARAPCurveDeformer*>();
			deformer->points = structure_curve->curve.mCtrlPoint;
		}

		if(node()->type() == Structure::SHEET)
		{
		}
	}
}

void Task::executeGrowShrink( double t )
{
	if(!isReady) return;

	Structure::Node * n = node();
	QVector<Structure::Link*> edges = getGoodEdges();

	/// Cut node cases:
	if( property.contains("isCutNode") )
	{
		if(n->type() == Structure::CURVE)
		{
			Structure::Curve* current_curve = ((Structure::Curve*)node());

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

				if(otherNode->type() == Structure::CURVE)
				{
					Structure::Curve* other_curve = ((Structure::Curve*) otherNode);

					int nCtrl = other_curve->curve.GetNumCtrlPoints();
					int idx_control = other_curve->controlPointIndexFromCoord( edge->getCoord(other_curve->id).front() );
					int idx_anchor = (idx_control > nCtrl / 2) ? 0 : nCtrl - 1;

					ARAPCurveDeformer * deformer = new ARAPCurveDeformer( other_curve->controlPoints() );

					deformer->ClearAll();
					deformer->setControl( idx_control );
					deformer->SetAnchor( idx_anchor );
					deformer->MakeReady();

					Vec3d newPosCtrl = edge->position(n->id);
					deformer->UpdateControl(idx_control, newPosCtrl);
					deformer->Deform( arapIterations );
					other_curve->setControlPoints( deformer->points );
				}

				if(n->type() == Structure::SHEET)
				{
				}
			}
		}

		if(n->type() == Structure::SHEET)
		{
		}

		return;
	}

	// Special case?
	if(edges.size() == 0){
		if(type == SHRINK){
			Structure::Link * singleEdge = active->getEdges(n->id).front();
			edges.push_back(singleEdge);
		}
	}

	/// Normal cases:

	// One edge
	if(edges.size() == 1)
	{
		if(node()->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)node());

			Array1D_Vector3 cpts = property["orgCtrlPoints"].value<Array1D_Vector3>();
			Array1D_Vector3 deltas = property["deltas"].value<Array1D_Vector3>();

			// Grow curve
			for(int u = 0; u < structure_curve->curve.mNumCtrlPoints; u++)
				structure_curve->curve.mCtrlPoint[u] = cpts[u] + (deltas[u] * t);	
		}

		if(node()->type() == Structure::SHEET)
		{
			Structure::Sheet* structure_sheet = ((Structure::Sheet*)node());

			Array2D_Vector3 cpts = property["orgCtrlPoints"].value<Array2D_Vector3>();
			Array2D_Vector3 deltas = property["deltas"].value<Array2D_Vector3>();

			// Grow sheet
			for(int u = 0; u < structure_sheet->surface.mNumUCtrlPoints; u++)
				for(int v = 0; v < structure_sheet->surface.mNumVCtrlPoints; v++)
					structure_sheet->surface.mCtrlPoint[u][v] = cpts[u][v] + (deltas[u][v] * t);
		}
	}

	if(edges.size() == 2)
	{
		if(node()->type() == Structure::CURVE)
		{
			executeMorph( t );
		}

		if(node()->type() == Structure::SHEET)
		{
		}
	}
}

void Task::prepareMorph()
{
	QVector<Structure::Link*> edges = getGoodEdges();

	if(property.contains("mergeTo") && edges.size())
	{
		foreach(Structure::Link* edge, edges)
		{
			Structure::Node * baseNode = edge->otherNode( node()->id );
			Structure::Node * mergeToNode = active->getNode(property["mergeTo"].toString());
			
			// Copy coordinates
			Structure::Link* toEdge = active->getEdge(mergeToNode->id, baseNode->id);

			Array1D_Vec4d coordOnBase = toEdge->getCoord(baseNode->id);
			Array1D_Vec4d coordOnMe = edge->getCoord(node()->id);

			Array1D_Vec4d c1 = (edge->n1->id == node()->id) ? coordOnMe : coordOnBase;
			Array1D_Vec4d c2 = (edge->n2->id == node()->id) ? coordOnMe : coordOnBase;

			edge->property["finalCoord_n1"].setValue( qMakePair(edge->n1->id, c1) );
			edge->property["finalCoord_n2"].setValue( qMakePair(edge->n2->id, c2) );
		}
	}

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		if(node()->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)node());

			Structure::Link * link = edges.front();
			Vec3d startPoint = link->position(node()->id);

			// Compute path
			GraphDistance graphDist(active, SingleNode(node()->id));
			QVector< NodeCoord > path;
			graphDist.computeDistances( link->positionOther( node()->id ) );
			graphDist.pathCoordTo( startPoint, path);

			property["path"].setValue( path );
			property["cpIDX"] = structure_curve->controlPointIndexFromCoord( link->getCoord(node()->id).front() );
		}

		if(node()->type() == Structure::SHEET)
		{
		}
	}

	// 2) TWO edges
	if(edges.size() == 2)
	{
		if(node()->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)node());

			// We will get two "handles" from furthest edges
			QList<Structure::Link*> farEdges = furthermostGoodEdges();

			// ARAP curve deformation
			setupCurveDeformer( structure_curve, farEdges.front(), farEdges.back() );
		}
	}

	// 3) More than two edges

}

void Task::executeMorph( double t )
{
	if(!isReady) return;

	Structure::Node * n = node();
	QVector<Structure::Link*> edges = getGoodEdges();

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		Structure::Link * edge = edges.front();

		if(n->type() == Structure::CURVE)
		{			
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
				
				int cpIDX_target = target_curve->controlPointIndexFromCoord(target->getEdge(tbaseNode, tnodeID)->getCoord(tnodeID).front());

				target_curve->curve.translateTo( newPos, cpIDX_target); // moves along for the ride..
			}

			// Move free end (linear interpolation)
			int freeEnd = (cpIDX < current_curve->curve.GetNumCtrlPoints() * 0.5) ? 
				current_curve->curve.GetNumCtrlPoints() - 1 : 0;

			Vec3d freeEndPos = AlphaBlend(pow(t,2), current_curve->controlPoint(freeEnd), target_curve->controlPoint(freeEnd));
			deformCurve(cpIDX, freeEnd, freeEndPos);
		}

		if(n->type() == Structure::SHEET)
		{

		}
	}

	// 2) TWO edges
	if(edges.size() == 2)
	{
		// Curve case:
		if(n->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)n);

			Structure::Link * linkA = property["linkA"].value<Structure::Link*>();
			Structure::Link * linkB = property["linkB"].value<Structure::Link*>();

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
			
			if(t == 1.0 && linkA->property.contains("finalCoord_n1"))
			{
				NodeCoords coordA = linkA->property["finalCoord_n1"].value<NodeCoords>();
				NodeCoords coordOtherA = linkA->property["finalCoord_n2"].value<NodeCoords>();
				if(coordA.first != n->id) std::swap(coordA, coordOtherA);

				NodeCoords coordB = linkB->property["finalCoord_n1"].value<NodeCoords>();
				NodeCoords coordOtherB = linkB->property["finalCoord_n2"].value<NodeCoords>();
				if(coordB.first != n->id) std::swap(coordB, coordOtherB);

				// Apply final coordinates
				linkA->setCoord(n->id, coordA.second);
				linkA->setCoordOther(n->id, coordOtherA.second);

				linkB->setCoord(n->id, coordB.second);
				linkB->setCoordOther(n->id, coordOtherB.second);
			}
		}

		// Sheet case:
		if(n->type() == Structure::SHEET)
		{

		}
	}

	// 3) More than two edges

	// Done with this morph
	if(t == 1.0)
	{
		foreach(Structure::Link * edge, active->getEdges(node()->id))
			edge->property["active"] = true;
	}
}

void Task::setupCurveDeformer( Structure::Curve* curve, Structure::Link* linkA, Structure::Link* linkB )
{
	std::vector<Vec3d> curvePoints = node()->controlPoints();
	
	if(property.contains("orgCtrlPoints"))
		curvePoints = property["orgCtrlPoints"].value< std::vector<Vec3d> >();

	property["deformer"].setValue( new ARAPCurveDeformer( curvePoints ) );

	property["linkA"].setValue( linkA );
	property["linkB"].setValue( linkB );

	property["cpidxA"].setValue( curve->controlPointIndexFromCoord( linkA->getCoord(node()->id).front() ) );
	property["cpidxB"].setValue( curve->controlPointIndexFromCoord( linkB->getCoord(node()->id).front() ) );
	
	// Compute paths for two ends
	Structure::Node * n = node();
	NodeCoords finalCoordA = linkA->property["finalCoord_n1"].value<NodeCoords>();
	NodeCoords finalCoordOtherA = linkA->property["finalCoord_n2"].value<NodeCoords>();
	if(finalCoordA.first != n->id) std::swap(finalCoordA, finalCoordOtherA);

	NodeCoords finaCoordB = linkB->property["finalCoord_n1"].value<NodeCoords>();
	NodeCoords finalCoordOtherB = linkB->property["finalCoord_n2"].value<NodeCoords>();
	if(finaCoordB.first != n->id) std::swap(finaCoordB, finalCoordOtherB);

	GraphDistance graphDistA(active, SingleNode(n->id));
	QVector< NodeCoord > pathA;
	graphDistA.computeDistances( linkA->otherNode(n->id)->position(finalCoordOtherA.second.front()) );
	graphDistA.pathCoordTo( linkA->position(n->id), pathA );

	//if(!node()->id.contains("_cloned"))
	//{
	//	foreach(NodeCoord path, pathA)
	//		target->debugPoints.push_back(active->position(path.first, path.second));

	//	target->debugPoints3.push_back(linkA->otherNode(n->id)->position(finalCoordOtherA.second.front()));
	//	target->debugPoints2.push_back(linkA->position(n->id));
	//}

	GraphDistance graphDistB(active, SingleNode(n->id));
	QVector< NodeCoord > pathB;
	graphDistB.computeDistances( linkB->otherNode(n->id)->position(finalCoordOtherB.second.front()) );
	graphDistB.pathCoordTo( linkB->position(n->id), pathB );

	// Save paths for A & B
	property["pathA"].setValue(pathA);
	property["pathB"].setValue(pathB);
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
		property["deformer"].setValue( deformer = new ARAPCurveDeformer( curve->curve.mCtrlPoint ) );
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
