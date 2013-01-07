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

		this->start = this->x();

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

void Task::execute()
{
	currentTime = start;

	switch(type)
	{
	case GROW:
	case SHRINK:
		prepareGrowShrink();
		executeGrowShrink();
		break;
	case SPLIT:
	case MERGE:
	case MORPH:
		prepareMorph();
		executeMorph();
		break;
	}
}

void Task::prepareGrowShrink()
{
	Structure::Node * n = node();
	QVector<Structure::Link*> edges = active->getEdges(n->id);

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

			if( !active->isCutNode(n->id) )
			{
				QList<Structure::Link*> farEdges = active->furthermostEdges( n->id );
				
				Structure::Link * linkA = farEdges.front();
				Structure::Link * linkB = farEdges.back();

				// ARAP curve deformation
				setupCurveDeformer( structure_curve, linkA, linkB );
				ARAPCurveDeformer* deformer = property["deformer"].value<ARAPCurveDeformer*>();

				Vec3d pointA = linkA->position( n->id );
				Vec3d pointB = linkB->position( n->id );

				GraphDistance gd( active, SingleNode(node()->id) );
				gd.computeDistances( pointA );
				QVector< NodeCoord > path;
				gd.pathCoordTo( pointB, path);

				NodeCoord midPointCoord = path[path.size() / 2];
				Vec3d midPoint = active->position(midPointCoord.first, midPointCoord.second);

				if(type == GROW)
				{
					structure_curve->foldTo( Vec4d(0.5), true );
					structure_curve->moveBy( midPoint - structure_curve->position(Vec4d(0.5)) );
					deformer->points = structure_curve->curve.mCtrlPoint;
				}
				
				if(type == SHRINK)
				{
					linkA->replace( linkA->otherNode(n->id)->id, active->getNode(midPointCoord.first), Array1D_Vec4d(1, midPointCoord.second));
					linkB->replace( linkB->otherNode(n->id)->id, active->getNode(midPointCoord.first), Array1D_Vec4d(1, midPointCoord.second));
				}

				// Morph operation need these
				linkA->property["finalCoord_n1"].setValue( qMakePair(linkA->n1->id, linkA->getCoord(linkA->n1->id)) );
				linkA->property["finalCoord_n2"].setValue( qMakePair(linkA->n2->id, linkA->getCoord(linkA->n2->id)) );

				linkB->property["finalCoord_n1"].setValue( qMakePair(linkB->n1->id, linkB->getCoord(linkB->n1->id)) );
				linkB->property["finalCoord_n2"].setValue( qMakePair(linkB->n2->id, linkB->getCoord(linkB->n2->id)) );
			}
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
	QVector<Structure::Link*> edges = active->getEdges(n->id);

	int stretchedLength = scaledTime();

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
				outGraphs.push_back( new Structure::Graph(*active) );
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
				outGraphs.push_back( new Structure::Graph(*active) );
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

	QVector<Structure::Link*> edges = active->getEdges(node()->id);

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
			QList<Structure::Link*> farEdges = active->furthermostEdges( node()->id );

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
	QVector<Structure::Link*> edges = active->getEdges(n->id);

	int stretchedLength = scaledTime();

	int arapIterations = 10;

	// 1) SINGLE edge
	if(edges.size() == 1)
	{
		if(n->type() == Structure::CURVE)
		{

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

					debugPoints.push_back(newPosA);
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

					debugPoints2.push_back(newPosB);
				}

				if(pathA.size() > 2 || pathB.size() > 2)
				{
					//QString fileName = QString("test_%1_Task%2_MorphCurve.xml").arg(globalCount++).arg(taskID);
					//active->saveToFile(fileName);

					outGraphs.push_back( new Structure::Graph(*active) );
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
	return currentTime >= start + length;
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
