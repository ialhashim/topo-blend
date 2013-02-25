#include <QApplication>

#include "Task.h"
#include "ARAPCurveDeformer.h"
#include "Scheduler.h"

#include "Synthesizer.h"
#include <QQueue>

Scheduler::Scheduler()
{
	rulerHeight = 25;
	sourceGraph = targetGraph = NULL;
}

void Scheduler::drawBackground( QPainter * painter, const QRectF & rect )
{
	QGraphicsScene::drawBackground(painter,rect);

	int y = rect.y();
	int screenBottom = y + rect.height();

	// Draw tracks
	for(int i = 0; i < (int)items().size() * 1.25; i++)
	{
		int y = i * 17;
		painter->fillRect(-10, y, 4000, 16, QColor(80,80,80));
	}

	// Draw current time indicator
	int ctime = slider->currentTime();
	painter->fillRect(ctime, 0, 1, screenBottom, QColor(0,0,0,128));
}

void Scheduler::drawForeground( QPainter * painter, const QRectF & rect )
{
	int x = rect.x();
	int y = rect.y();

	// Draw ruler
	int screenBottom = y + rect.height();
	painter->fillRect(x, screenBottom - rulerHeight, rect.width(), rulerHeight, QColor(64,64,64));

	// Draw yellow line
	int yellowLineHeight = 2;
	painter->fillRect(x, screenBottom - rulerHeight - yellowLineHeight, rect.width(), yellowLineHeight, Qt::yellow);

	// Draw text & ticks
	int totalTime = totalExecutionTime();
	int spacing = totalTime / 10;
	int timeEnd = 10;
	int minorTicks = 5;
	painter->setPen(Qt::gray);
	QFontMetrics fm(painter->font());

	for(int i = 0; i <= timeEnd; i++)
	{
		double time = double(i) / timeEnd;

		int curX = i * spacing;

		QString tickText = QString("00:%1").arg(time);
		painter->drawText(curX - (fm.width(tickText) * 0.5), screenBottom - 14, tickText );

		// Major tick
		painter->drawLine(curX, screenBottom, curX, screenBottom - 10);

		if(i != timeEnd)
		{
			// Minor tick
			for(int j = 1; j < minorTicks; j++)
			{
				double delta = double(spacing) / minorTicks;
				int minorX = curX + (j * delta);
				painter->drawLine(minorX, screenBottom, minorX, screenBottom - 5);
			}
		}
	}

	slider->forceY(screenBottom - rulerHeight - 10);
	slider->setY(slider->myY);
	painter->translate(slider->pos());
	slider->paint(painter, 0, 0);
}

void Scheduler::schedule()
{
	Task *current, *prev = NULL;

	foreach(Task * task, tasks)
	{
		// Create
		current = task;

		// Placement
		if(prev) current->moveBy(prev->x() + prev->width,prev->y() + (prev->height));
		current->currentTime = current->x();
		current->start = current->x();

		// Add to scene
		this->addItem( current );

		prev = current;
	}

	// Order and group here:
	this->order();

	// Time-line slider
	slider = new TimelineSlider;
	slider->reset();
	this->connect( slider, SIGNAL(timeChanged(int)), SLOT(timeChanged(int)) );
    this->addItem( slider );
}

void Scheduler::order()
{
	QMultiMap<Task::TaskType,Task*> tasksByType;

	foreach(Task * task, tasks)
		tasksByType.insert(task->type, task);

	int curStart = 0;

	// General layout
	for(int i = Task::SHRINK; i <= Task::GROW; i++)
	{
		QList<Task*> curTasks = tasksByType.values(Task::TaskType(i));
		if(!curTasks.size()) continue;

		// Sort tasks by priority
		curTasks = sortTasksByPriority( curTasks );

		int futureStart = curStart;

		if(i == Task::MORPH)
		{
			foreach(Task* t, curTasks){
				t->setStart(curStart);
				futureStart = qMax(futureStart, t->endTime());
				curStart = futureStart;
			}
		}
		else
		{
			foreach(Task* t, curTasks){
				t->setStart(curStart);
				futureStart = qMax(futureStart, t->endTime());
			}
		}

		curStart = futureStart;
	}

	// Collect morph generated from split together
	foreach(Task* t, tasks)
	{
		if(t->type == Task::SPLIT)
		{
			Task * morphTask = getTaskFromNodeID( t->property["splitFrom"].toString() );
			morphTask->setStart( t->start );
		}

		if(t->type == Task::MERGE)
		{
			Task * morphTask = getTaskFromNodeID( t->property["mergeTo"].toString() );
			morphTask->setStart( t->start );
		}
	}
}

void Scheduler::executeAll()
{
	qApp->setOverrideCursor(Qt::WaitCursor);

	emit( progressStarted() );

	double timeStep = 0.01;
	int totalTime = totalExecutionTime();

	QVector<Task*> allTasks = tasksSortedByStart();

	// Early preparations needed for cut nodes
	for(int i = 0; i < (int)allTasks.size(); i++)
	{
		Task * task = allTasks[i];

		if( task->type == Task::GROW && targetGraph->isCutNode(task->targetNode()->id) )
		{
			task->prepare();
			task->property["cutNodeGrow"] = true;
		}

		if( task->type == Task::SHRINK && sourceGraph->isCutNode(task->node()->id) )
		{
			task->property["cutNodeShrink"] = true;
		}
	}

	// Execute all tasks
	for(double globalTime = 0; globalTime <= (1.0 + timeStep); globalTime += timeStep)
	{
		QElapsedTimer timer; timer.start();

		// DEBUG - per pass
		activeGraph->debugPoints.clear();

		activeGraph->setPropertyAll("isActive", false);

		for(int i = 0; i < (int)allTasks.size(); i++)
		{
			Task * task = allTasks[i];
			double localTime = task->localT( globalTime * totalTime );
			task->node()->property["t"] = localTime;
	
			QVector<QString> rtasks = activeTasks(globalTime * totalTime);
			activeGraph->property["running_tasks"].setValue( rtasks );

			if( localTime < 0 || task->isDone ) continue;

			prepare_relink( task );

			task->execute( localTime );

			relink( task );

			if(localTime >= 0.0 && localTime < 1.0) task->node()->property["isActive"] = true;
		}

		// Geometry morphing
		foreach(Task * task, allTasks){
			double localTime = task->localT( globalTime * totalTime );
			if( localTime < 0 ) continue;
			task->geometryMorph( localTime );
		}
		
		// Output current active graph:
		allGraphs.push_back( new Structure::Graph( *activeGraph ) );

		// UI - visual indicator:
		int percent = globalTime * 100;
		emit( progressChanged(percent) );
	}

	slider->enable();

	emit( progressDone() );

	qApp->restoreOverrideCursor();
}

void Scheduler::prepare_relink( Task * task )
{
	if(task->property.contains("linkDeltas")) return;

	Structure::Node * n = task->node();
	QVector<Structure::Link*> edges = activeGraph->getEdges(n->id);

	LinksDelta deltas;

	if( task->property.contains("cutNodeGrow") )
	{
		foreach(Structure::Link* link, edges)
		{
			Structure::Node * other = link->otherNode(n->id);
			Task * otherTask = getTaskFromNodeID(other->id);

			otherTask->isDone = false;
		}
	}

	foreach(Structure::Link* link, edges)
	{
		Structure::Node * other = link->otherNode(n->id);
		Task * otherTask = getTaskFromNodeID(other->id);

		if( !otherTask->isDone )
		{
			Vec3d delta = link->positionOther(n->id) - link->position(n->id);
			if(delta.norm() < 1e-7) delta = Vector3(0);

			deltas[ link ] = delta;
		}
	}

	task->property["linkDeltas"].setValue( deltas );
}

void Scheduler::relink( Task * task )
{
	LinksDelta deltas = task->property["linkDeltas"].value<LinksDelta>();
	if( !deltas.size() ) return;

	if( task->type != Task::MORPH && 
		!task->property.contains("cutNodeGrow") && 
		!task->property.contains("cutNodeShrink"))
		return;

	Structure::Node * n = task->node();

	foreach(Structure::Link * link, deltas.keys())
	{
		if( !link->hasNode(n->id) ) continue;

		Structure::Node * other = link->otherNode(n->id);

		Vec4d handle = link->getCoordOther(n->id).front();

		Vector3 delta = deltas[link];
		Vector3 newPos = link->position(n->id) + delta;

		other->deformTo( handle, newPos );
	}
}

void Scheduler::drawDebug()
{
	foreach(Task * t, tasks)
		t->drawDebug();
}

int Scheduler::totalExecutionTime()
{
	int endTime = 0;

	foreach(Task * t, tasks)
		endTime = qMax(endTime, t->endTime());

	return endTime;
}

void Scheduler::timeChanged( int newTime )
{
	int idx = allGraphs.size() * (double(newTime) / totalExecutionTime());

	idx = qRanged(0, idx, allGraphs.size() - 1);

	emit( activeGraphChanged(allGraphs[idx]) );
}

void Scheduler::doBlend()
{
	emit( startBlend() );
}

QVector<Task*> Scheduler::tasksSortedByStart()
{
	QMap<Task*,int> tasksMap;
	typedef QPair<int, Task*> IntTaskPair;
	foreach(Task* t, tasks) tasksMap[t] = t->start;
	QList< IntTaskPair > sortedTasksList = sortQMapByValue<Task*,int>( tasksMap );
	QVector< Task* > sortedTasks; 
	foreach( IntTaskPair p, sortedTasksList ) sortedTasks.push_back(p.second);
	return sortedTasks;
}

void Scheduler::stopExecution()
{
	isForceStop = true;
}

void Scheduler::startAllSameTime()
{
	foreach(Task * t, tasks)
		t->setX(0);
}

void Scheduler::prepareSynthesis()
{

}

Task * Scheduler::getTaskFromNodeID( QString nodeID )
{
	foreach(Task * t, tasks) if(t->node()->id == nodeID) return t;
	return NULL;
}

QVector<QString> Scheduler::activeTasks( double globalTime )
{
	QVector<QString> aTs;

	for(int i = 0; i < (int)tasks.size(); i++)
	{
		Task * task = tasks[i];
		double localTime = task->localT( globalTime );
		if ( task->isActive( localTime ) )
		{
			QString nodeID = task->node()->id;
			aTs.push_back(nodeID);
		}
	}

	return aTs;
}

void Scheduler::relink( double t )
{
	QVector<QString> currTasks = activeTasks(t);

	// Past code
	/*foreach(QString curr_nid, currTasks)
	{
		Task *task = getTaskFromNodeID(curr_nid);

		if (task->property.contains("isConstraint"))
		{
			if (task->property["isConstraint"].toBool())
				relinkConstraintNode(curr_nid);
			else
				relinkFreeNode(curr_nid);
		}
	}*/
}

void Scheduler::relinkConstraintNode(QString cnID)
{
	// task is seed and relink all other nodes
	// set up default tags
	foreach(Structure::Node *node, activeGraph->nodes)
		node->property["fixed"] = false;

	// BFS
	QQueue<QString> nodesQ;
	nodesQ.enqueue(cnID);
	while(!nodesQ.empty())
	{
		QString nodeID  = nodesQ.dequeue();
		Structure::Node *node = activeGraph->getNode(nodeID);

		// get fixed neighbors
		// add non-fixed neighbors into queue
		QVector<QString> fixedNeighbors;
		foreach(Structure::Link* link, activeGraph->getEdges(nodeID))
		{
			Structure::Node* other = link->otherNode(nodeID);
			if (other->property["fixed"].toBool())
				fixedNeighbors.push_back(other->id);
			else
				nodesQ.enqueue(other->id);
		}

		// fix node according to fixed neighbors
		if (!fixedNeighbors.empty())
		{
			if (node->type() == Structure::CURVE)
			{
				// move one end
				if (fixedNeighbors.size() == 1)
				{
					Structure::Link* link = activeGraph->getEdge(nodeID, fixedNeighbors.front());
					moveNodeByLink(node, link);
				}

				// move two ends
				if (fixedNeighbors.size() == 2)
				{
					Structure::Link *linkA = activeGraph->getEdge(nodeID, fixedNeighbors.front());
					Structure::Link *linkB = activeGraph->getEdge(nodeID, fixedNeighbors.back());
					deformCurveByLink(node, linkA);
					deformCurveByLink(node, linkB);
				}
			}

			if (node->type() == Structure::SHEET)
			{
				// One end translate
				if (fixedNeighbors.size() == 1)
				{
					Structure::Link* link = activeGraph->getEdge(nodeID, fixedNeighbors.front());
					moveNodeByLink(node, link);
				}

				// Two ends translate to match middle point
				if (fixedNeighbors.size() == 2)
				{
					Structure::Link *linkA = activeGraph->getEdge(nodeID, fixedNeighbors.front());
					Structure::Link *linkB = activeGraph->getEdge(nodeID, fixedNeighbors.back());
					tranformSheetByTwoLinks(node, linkA, linkB);
				}
			}
		}

		// fixed
		activeGraph->getNode(nodeID)->property["fixed"] = true;
	}
}

void Scheduler::relinkFreeNode( QString fnID )
{
	Structure::Node *node = activeGraph->getNode(fnID);
	QVector<Structure::Link*> edges = activeGraph->getEdges(fnID);

	if (node->type() == Structure::SHEET)
	{
		// One end translate
		if (edges.size() == 1)
		{
			Structure::Link* link = edges.front();
			moveNodeByLink(node, link);
		}

		// Two ends translate to match middle point
		if (edges.size() == 2)
		{
			Structure::Link *linkA = edges.front();
			Structure::Link *linkB = edges.back();
			tranformSheetByTwoLinks(node, linkA, linkB);
		}
	}
}


void Scheduler::moveNodeByLink(Structure::Node* node, Structure::Link *link)
{
	Vec3d oldPosition = link->position(node->id);
	Vec3d newPosition = link->positionOther(node->id);

	node->moveBy(newPosition - oldPosition);
}

void Scheduler::deformCurveByLink( Structure::Node* node, Structure::Link *link )
{
	Structure::Curve * curve = (Structure::Curve *)node;

	//int cpidxAnchor = curve->controlPointIndexFromCoord( link->getCoord(node->id).front() );

	// Move free end (linear interpolation)
    //int cpidxControl = (cpidxAnchor < curve->curve.GetNumCtrlPoints() * 0.5) ? curve->curve.GetNumCtrlPoints() - 1 : 0;
    //Vec3d newPosition = link->positionOther(node->id);
}

void Scheduler::tranformSheetByTwoLinks( Structure::Node* node, Structure::Link *linkA, Structure::Link *linkB )
{
	Vec3d oldPositionA = linkA->position(node->id);
	Vec3d newPositionA = linkA->positionOther(node->id);

	Vec3d oldPositionB = linkB->position(node->id);
	Vec3d newPositionB = linkB->positionOther(node->id);

	Vec3d oldMiddle = (oldPositionA + oldPositionB) / 2;
	Vec3d newMiddle = (newPositionA + newPositionB) / 2;

	node->moveBy(newMiddle - oldMiddle);
}

QList<Task*> Scheduler::sortTasksByPriority( QList<Task*> currentTasks )
{
	QList<Task*> sorted;

	// Sort by type: Sheets before curves
	QMap<Task*,int> curveTasks, sheetTasks;
	foreach(Task* t, currentTasks)
	{
		if(!t->node()) continue;

		if(t->node()->type() == Structure::CURVE) 
			curveTasks[t] = activeGraph->valence(t->node());
		if(t->node()->type() == Structure::SHEET) 
			sheetTasks[t] = activeGraph->valence(t->node());
	}

	// Sort by valence: Highly connected before individuals
	QList< QPair<int, Task*> > sortedCurve = sortQMapByValue(curveTasks);
	QList< QPair<int, Task*> > sortedSheet = sortQMapByValue(sheetTasks);

	// Combine: Curve[low] --> Curve[high] + Sheet[low] --> Sheet[heigh]
	for (int i = 0; i < (int)sortedCurve.size(); ++i) sorted.push_back( sortedCurve.at(i).second );
	for (int i = 0; i < (int)sortedSheet.size(); ++i) sorted.push_back( sortedSheet.at(i).second );

	// Reverse
	for(int k = 0; k < (sorted.size()/2); k++) sorted.swap(k,sorted.size()-(1+k));

	return sorted;
}

