#include "Scheduler.h"
#include "Task.h"
#include "ARAPCurveDeformer.h"
#include <QGraphicsSceneMouseEvent>

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

		setStart(newPos.x());

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

int Task::scaledTime()
{
	return length / 10;
}

void Task::prepare()
{
	currentTime = start;

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
}

void Task::execute()
{	
	currentTime = start;

	// Fill with start graph
	outGraphs.resize(scaledTime());
	for(int i = 0; i < scaledTime(); i++) outGraphs[i] = new Structure::Graph(*active);

	switch(type)
	{
	case GROW:
	case SHRINK:
		executeGrowShrink();
		break;
	case SPLIT:
	case MERGE:
	case MORPH:
		executeMorph();
		break;
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

		if(n->type() == Structure::SHEET)
		{
		}

		this->isReady = true;

		return;
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

			QList<Structure::Link*> farEdges = furthermostGoodEdges();

			Structure::Link * linkA = farEdges.front();
			Structure::Link * linkB = farEdges.back();

			// ARAP curve deformation
			setupCurveDeformer( structure_curve, linkA, linkB );
			ARAPCurveDeformer* deformer = property["deformer"].value<ARAPCurveDeformer*>();
			
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
				deformer->points = structure_curve->curve.mCtrlPoint;
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
		}

		if(node()->type() == Structure::SHEET)
		{
		}
	}

	this->isReady = true;
}

void Task::executeGrowShrink()
{
	if(!isReady) return;

	Structure::Node * n = node();
	QVector<Structure::Link*> edges = getGoodEdges();

	int stretchedLength = scaledTime();

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

			for(int i = 0; i < stretchedLength; i++)
			{
				double t = (double)i / (stretchedLength - 1);

				// Grow / shrink curve
				for(int u = 0; u < current_curve->curve.mNumCtrlPoints; u++)
					current_curve->curve.mCtrlPoint[u] = cpts[u] + (deltas[u] * t);

				// Relink:
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

						debugPoints.push_back(newPosCtrl);
					}

					if(n->type() == Structure::SHEET)
					{
					}
				}

				outGraphs[i] = new Structure::Graph(*active);
			}
		}

		if(n->type() == Structure::SHEET)
		{
		}

		return;
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

			for(int i = 0; i < stretchedLength; i++)
			{
				double t = (double)i / (stretchedLength - 1);

				// Grow curve
				for(int u = 0; u < structure_curve->curve.mNumCtrlPoints; u++)
					structure_curve->curve.mCtrlPoint[u] = cpts[u] + (deltas[u] * t);

				//active->saveToFile(QString("test_growCurve_0%1.xml").arg(i));
				outGraphs[i] = new Structure::Graph(*active);
			}
		}

		if(node()->type() == Structure::SHEET)
		{
			Structure::Sheet* structure_sheet = ((Structure::Sheet*)node());

			Array2D_Vector3 cpts = property["orgCtrlPoints"].value<Array2D_Vector3>();
			Array2D_Vector3 deltas = property["deltas"].value<Array2D_Vector3>();

			for(int i = 0; i < stretchedLength; i++)
			{
				double t = (double)i / (stretchedLength - 1);

				// Grow sheet
				for(int u = 0; u < structure_sheet->surface.mNumUCtrlPoints; u++)
					for(int v = 0; v < structure_sheet->surface.mNumVCtrlPoints; v++)
						structure_sheet->surface.mCtrlPoint[u][v] = cpts[u][v] + (deltas[u][v] * t);

				//active->saveToFile(QString("test_growSheet_%1.xml").arg(i));
				outGraphs[i] = new Structure::Graph(*active);
			}
		}
	}

	if(edges.size() == 2)
	{
		if(node()->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)n);

			executeMorph();
		}

		if(node()->type() == Structure::SHEET)
		{
		}
	}
}

void Task::prepareMorph()
{
	isReady = true;

	QVector<Structure::Link*> edges = getGoodEdges();

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		if(node()->type() == Structure::CURVE)
		{
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

void Task::executeMorph()
{
	if(!isReady) return;

	Structure::Node * n = node();
	QVector<Structure::Link*> edges = getGoodEdges();

	int stretchedLength = scaledTime();

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		if(n->type() == Structure::CURVE)
		{			
			Structure::Curve* current_curve = ((Structure::Curve*)n);

			Structure::Link * link = edges.front();

			GraphDistance graphDist(active, SingleNode(n->id));
			QVector< NodeCoord > path;

			Vec3d startPoint = link->position(n->id);

			graphDist.computeDistances( link->positionOther( n->id ) );
			graphDist.pathCoordTo( startPoint, path);

			for(int i = 0; i < stretchedLength; i++)
			{
				double t = (double)i / (stretchedLength - 1);

				int current = t * (path.size() - 1);

				Vec3d delta = active->position(path[current].first,path[current].second) - startPoint;
				current_curve->curve.translate( delta );

				outGraphs[i] = new Structure::Graph(*active);

				if (t < 1.0)
					current_curve->curve.translate( -delta );
			}
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

			// Retrieve final coordinates
			NodeCoords coordA = linkA->property["finalCoord_n1"].value<NodeCoords>();
			NodeCoords coordOtherA = linkA->property["finalCoord_n2"].value<NodeCoords>();
			if(coordA.first != n->id) std::swap(coordA, coordOtherA);

			NodeCoords coordB = linkB->property["finalCoord_n1"].value<NodeCoords>();
			NodeCoords coordOtherB = linkB->property["finalCoord_n2"].value<NodeCoords>();
			if(coordB.first != n->id) std::swap(coordB, coordOtherB);

			GraphDistance graphDistA(active, SingleNode(n->id));
			QVector< NodeCoord > pathA;
			graphDistA.computeDistances( linkA->otherNode(n->id)->position(coordOtherA.second.front()) );
			graphDistA.pathCoordTo( linkA->position(n->id), pathA);

			GraphDistance graphDistB(active, SingleNode(n->id));
			QVector< NodeCoord > pathB;
			graphDistB.computeDistances( linkB->otherNode(n->id)->position(coordOtherB.second.front()) );
			graphDistB.pathCoordTo( linkB->position(n->id), pathB );

			for(int i = 0; i < stretchedLength; i++)
			{
				double t = (double)i / (stretchedLength - 1);

				int idxA = t * (pathA.size() - 1);
				int idxB = t * (pathB.size() - 1);

				// Move point A to next step
				if(pathA.size() > 2)
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

					//debugPoints.push_back(newPosA);
				}

				// Move point B to next step
				if(pathB.size() > 2)
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

					//debugPoints2.push_back(newPosB);
				}

				// Output the graph:
				if(pathA.size() > 2 || pathB.size() > 2)
				{
					outGraphs[i] = new Structure::Graph(*active);
				}
			}

			// Apply final coordinates
			linkA->setCoord(n->id, coordA.second);
			linkA->setCoordOther(n->id, coordOtherA.second);

			linkB->setCoord(n->id, coordB.second);
			linkB->setCoordOther(n->id, coordOtherB.second);
		}

		// Sheet case:
		if(n->type() == Structure::SHEET)
		{

		}
	}

	// 3) More than two edges
}

void Task::setupCurveDeformer( Structure::Curve* curve, Structure::Link* linkA, Structure::Link* linkB )
{
	property["linkA"].setValue( linkA );
	property["linkB"].setValue( linkB );

	property["cpidxA"].setValue( curve->controlPointIndexFromCoord( linkA->getCoord(node()->id).front() ) );
	property["cpidxB"].setValue( curve->controlPointIndexFromCoord( linkB->getCoord(node()->id).front() ) );

	std::vector<Vec3d> ctrlPoints = curve->curve.mCtrlPoint;
	property["deformer"].setValue( new ARAPCurveDeformer(ctrlPoints, ctrlPoints.size() * 0.25) );
}

Structure::Node * Task::node()
{
	return active->getNode(this->property["nodeID"].toString());
}

bool Task::stillWorking()
{
	return currentTime == start + length;
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

void Task::setStart( int newStart )
{
	start = newStart;
	this->setX(newStart);
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
