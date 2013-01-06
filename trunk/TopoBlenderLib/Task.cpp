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
	this->current = start;
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
	painter->drawText(boundingRect(), QString("Len: %1").arg(width), QTextOption(Qt::AlignVCenter | Qt::AlignHCenter));
}

QVariant Task::itemChange( GraphicsItemChange change, const QVariant & value )
{
	if (scene() && change == ItemPositionChange && !isResizing) 
	{
		QPointF newPos = value.toPointF();

		newPos.setX(qMax(0.0,newPos.x()));
		newPos.setY(this->y());

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
	return length / 20;
}

void Task::execute()
{
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

	this->isReady = true;
}

void Task::executeGrowShrink()
{
	if(!isReady) return;

	Structure::Node * n = node();
	QVector<Structure::Link*> edges = active->getEdges(n->id);

	int stretchedLength = scaledTime();

	for(int i = 0; i < stretchedLength; i++)
	{
		double t = (double)i / (stretchedLength - 1);

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

				active->saveToFile(QString("test_growCurve_0%1.xml").arg(i));
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

				active->saveToFile(QString("test_growSheet_%1.xml").arg(i));
			}
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

			property["linkA"].setValue( farEdges.first() );
			property["linkB"].setValue( farEdges.last() );

			property["cpidxA"].setValue( structure_curve->controlPointIndexFromCoord( farEdges.first()->getCoord(node()->id).front() ) );
			property["cpidxB"].setValue( structure_curve->controlPointIndexFromCoord( farEdges.last()->getCoord(node()->id).front() ) );

			std::vector<Vec3d> ctrlPoints = structure_curve->curve.mCtrlPoint;
			property["deformer"].setValue( new ARAPCurveDeformer(ctrlPoints, ctrlPoints.size() * 0.25) );
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

			ARAPCurveDeformer * deformer = property["deformer"].value<ARAPCurveDeformer*>();

			int cpidxA = property["cpidxA"].toInt();
			int cpidxB = property["cpidxB"].toInt();

			Array1D_Vec4d coordOtherA = (linkA->n1->id == n->id) ? 
				linkA->property["n2_newCoord"].value<Array1D_Vec4d>() 
				: linkA->property["n1_newCoord"].value<Array1D_Vec4d>();

			Array1D_Vec4d coordOtherB = (linkB->n1->id == n->id) ? 
				linkB->property["n2_newCoord"].value<Array1D_Vec4d>() 
				: linkB->property["n1_newCoord"].value<Array1D_Vec4d>();

			GraphDistance graphDist(active);

			QVector< NodeCoord > pathA;
			graphDist.computeDistances( linkA->otherNode(n->id)->position(coordOtherA.front()) );
			graphDist.pathCoordTo( linkA->position(n->id), pathA);

			QVector< NodeCoord > pathB;
			graphDist.computeDistances( linkB->otherNode(n->id)->position(coordOtherB.front()) );
			graphDist.pathCoordTo( linkB->position(n->id), pathB );

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

				QString fileName = QString("test_%1_Task%2_MorphCurve.xml").arg(globalCount++).arg(taskID);
				active->saveToFile(fileName);
			}

			// Update to new coordinates
			linkA->coord[0] = linkA->property["n1_newCoord"].value<Array1D_Vec4d>();
			linkA->coord[1] = linkA->property["n2_newCoord"].value<Array1D_Vec4d>();

			linkB->coord[0] = linkB->property["n1_newCoord"].value<Array1D_Vec4d>();
			linkB->coord[1] = linkB->property["n2_newCoord"].value<Array1D_Vec4d>();
		}

		// Sheet case:
		if(n->type() == Structure::SHEET)
		{

		}
	}

	// 3) More than two edges
}

Structure::Node * Task::node()
{
	return active->getNode(this->property["nodeID"].toString());
}

bool Task::stillWorking()
{
	return current >= start + length;
}

void Task::reset()
{
	isReady = false;
	current = start;
}
