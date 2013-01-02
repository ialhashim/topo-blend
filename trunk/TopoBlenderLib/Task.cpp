#include "Scheduler.h"
#include "Task.h"
#include "FFD.h"
#include <QGraphicsSceneMouseEvent>

typedef std::vector< std::pair<double,double> > VectorPairDouble;
Q_DECLARE_METATYPE(Vector3);
Q_DECLARE_METATYPE(VectorPairDouble);

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
		prepareSplitMerge();
		executeSplitMerege();
		break;
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

	int stretchedLength = length / 20;

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
	this->isReady = true;
}

void Task::executeMorph()
{
	if(!isReady) return;
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

void Task::prepareSplitMerge()
{
	isReady = true;

	Structure::Node * n = node();
	QVector<Structure::Link*> edges = active->getEdges(n->id);

	if(edges.size() > 1)
	{
		if(node()->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)n);

			// We will get two "handles" from furthest edges
			QList<Structure::Link> farEdges = active->furthermostEdges( n->id );
			Structure::Link linkA = farEdges.first();
			Structure::Link linkB = farEdges.last();

			Vec3d posA = linkA.position(n->id);
			Vec3d posB = linkB.position(n->id);
			Vec3d d = posB - posA;
			Vec3d midPoint = 0.5 * (posA + posB);
			Vec3d normal = structure_curve->curve.GetNormal(0.5).normalized();

			VectorPairDouble params;

			for(int i = 0; i < (int) structure_curve->curve.mCtrlPoint.size(); i++)
			{
				Vec3d & cp = structure_curve->curve.mCtrlPoint[i];
				Vec3d delta = cp - midPoint;

				double angle = -signedAngle(d.normalized(), delta.normalized(), cross(d.normalized(), delta.normalized()));
				double dist = delta.norm();

				params.push_back(std::make_pair(dist,angle));
			}

			property["normal"].setValue(normal);
			property["wire"].setValue(params);
			property["linkA"].setValue(linkA);
			property["linkB"].setValue(linkB);
		}
	}
}

void Task::executeSplitMerege()
{
	if(!isReady) return;

	Structure::Node * n = node();
	QVector<Structure::Link*> edges = active->getEdges(n->id);

	int stretchedLength = length / 20;

	active->saveToFile("zBefore.xml");

	if(edges.size() > 1)
	{
		if(n->type() == Structure::CURVE)
		{
			Structure::Curve* structure_curve = ((Structure::Curve*)n);

			GraphDistance graphDist(active);

			Structure::Link linkA = property["linkA"].value<Structure::Link>();
			Structure::Link linkB = property["linkB"].value<Structure::Link>();

			Vec3d from[2] = { linkA.positionOther(n->id), linkB.positionOther(n->id) };
			Vec3d to[2] = { linkA.position(n->id), linkB.position(n->id) };

			QVector< NodeCoord > pathA;
			graphDist.computeDistances( from[0] );
			graphDist.pathCoordTo(to[0], pathA);

			QVector< NodeCoord > pathB;
			graphDist.computeDistances( from[1] );
			graphDist.pathCoordTo(to[1], pathB);

			if(type == MERGE)
			{
				//std::reverse(pathA.begin(),pathA.end());
				//std::reverse(pathB.begin(),pathB.end());
			}

			VectorPairDouble params = property["wire"].value<VectorPairDouble>();
			Vector3 normal = property["normal"].value<Vector3>();

			for(int i = 0; i < stretchedLength; i++)
			{
				double t = (double)i / (stretchedLength - 1);

				int idxA = t * (pathA.size() - 1);
				int idxB = t * (pathB.size() - 1);

				// Reconstruct curve
				Vec3d posA = active->position(pathA[idxA].first, pathA[idxA].second);
				Vec3d posB = active->position(pathB[idxB].first, pathB[idxB].second);
				Vec3d d = posB - posA;
				Vec3d midPoint = 0.5 * (posA + posB);

				debugPoints.push_back(posA);
				debugPoints2.push_back(posB);

				for(int c = 0; c < (int) structure_curve->curve.mCtrlPoint.size(); c++)
				{
					double dist = params[c].first;
					double angle = params[c].second;
					structure_curve->curve.mCtrlPoint[c] = midPoint + (dist * rotatedVec(d.normalized(), angle, 
						cross(d.normalized(), (structure_curve->curve.mCtrlPoint[c]-midPoint).normalized())));
				}

				QString fileName = QString("test_splitCurve_%1_%2.xml").arg(taskID).arg(i);
				active->saveToFile(fileName);
			}
		}
	}
}
