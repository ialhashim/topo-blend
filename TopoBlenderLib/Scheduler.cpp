#include <QApplication> // For mouse icon changing
#include <QtConcurrentRun> // For easy multi-threading

#include "TaskCurve.h"
#include "TaskSheet.h"
#include "Scheduler.h"
#include "Relink.h"

#include "Synthesizer.h"
#include <QQueue>

#include "TaskGroups.h"

#include "Scheduler.h"
#include "SchedulerWidget.h"

Q_DECLARE_METATYPE( QSet<int> ) // for tags

	Scheduler::Scheduler() : globalStart(0.0), globalEnd(1.0), timeStep( 1.0 / 100.0 ), overTime(0.0), isApplyChangesUI(false)
{
	rulerHeight = 25;

	originalActiveGraph = originalTargetGraph = NULL;
	activeGraph = targetGraph = NULL;
	slider = NULL;
	widget = NULL;
}

Scheduler::~Scheduler()
{
	qDeleteAll(allGraphs);
	allGraphs.clear();

	qDeleteAll(tasks);
	tasks.clear();

	if(widget) delete widget;
	if(slider) delete slider;

	delete originalActiveGraph;
	delete originalTargetGraph;

	delete activeGraph;
	delete targetGraph;
}

Scheduler::Scheduler( const Scheduler& other )
{
	// Properties
	rulerHeight = other.rulerHeight;
	isForceStop = other.isForceStop;
	property = other.property;
	isApplyChangesUI = other.isApplyChangesUI;

	// UI elements
	widget = NULL;
	slider = NULL;
	dock = NULL;

	// Execution parameters
	timeStep = other.timeStep;
	globalStart = other.globalStart;
	globalEnd = other.globalEnd;
	overTime = other.overTime;

	// Input
	setInputGraphs( other.originalActiveGraph, other.originalTargetGraph );
	superNodeCorr = other.superNodeCorr;
	activeGraph = NULL;
	targetGraph = NULL;

	this->generateTasks();
	this->schedule();
}

Scheduler * Scheduler::clone()
{
	return new Scheduler(*this);
}

void Scheduler::drawBackground( QPainter * painter, const QRectF & rect )
{
	QGraphicsScene::drawBackground(painter,rect);

	int y = rect.y();
	int screenBottom = y + rect.height();

	// Draw tracks
	for(int i = 0; i < (int)tasks.size() * 1.25; i++)
	{
		int y = i * 17;
		painter->fillRect(-10, y, 4000, 16, QColor(80,80,80));
	}

	// Draw current time indicator
	int ctime = slider->currentTime();
	painter->fillRect(ctime, 0, 1, screenBottom, QColor(0,0,0,128));

	// Draw tags for interesting topology changes
	{
		int tagWidth = 5;
		QSet<int> timeTags = property["timeTags"].value< QSet<int> >();

		foreach(int tagTime, timeTags)
		{
			painter->fillRect(tagTime - (0.5 * tagWidth), 0, tagWidth, screenBottom, QColor(100,100,100,128));
		}
	}
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

	if(overTime)
	{
		QRectF rect(0, 0, tasks.size() * tasks.front()->height, Task::DEFAULT_LENGTH);
		QPointF startPos = rect.topLeft() + QPointF(totalTime - overTime + rect.height(),0);

		painter->save();
		painter->translate(startPos);
		painter->rotate(90);
		painter->setBrush(QColor(255,255,0,20));
		painter->drawRect(rect);
		painter->drawText(rect, Qt::AlignCenter, "OVERTIME");
		painter->restore();
	}

	for(int i = 0; i <= timeEnd; i++)
	{
		double time = double(i) / timeEnd;

		int curX = i * spacing;

		QString tickText = QString("%1").arg(time);
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

void Scheduler::setInputGraphs( Structure::Graph * source, Structure::Graph * target )
{
	originalActiveGraph = new Structure::Graph(*source);
	originalTargetGraph = new Structure::Graph(*target);
}

void Scheduler::generateTasks()
{
	tasks.clear();

	if( activeGraph ) delete activeGraph;
	if( targetGraph ) delete targetGraph;

	this->activeGraph = new Structure::Graph(*originalActiveGraph);
	this->targetGraph = new Structure::Graph(*originalTargetGraph);

	foreach(QString snodeID, superNodeCorr.keys())
	{
		QString tnodeID = superNodeCorr[snodeID];

		Task * task;

		if(activeGraph->getNode(snodeID)->type() == Structure::CURVE)
		{
			if (snodeID.contains("_null"))  // Grow
				task = new TaskCurve( activeGraph, targetGraph, Task::GROW, tasks.size() );
			else if (tnodeID.contains("_null")) // Shrink
				task = new TaskCurve( activeGraph, targetGraph, Task::SHRINK, tasks.size() );
			else
				task = new TaskCurve( activeGraph, targetGraph, Task::MORPH, tasks.size() );
		}

		if(activeGraph->getNode(snodeID)->type() == Structure::SHEET)
		{
			if (snodeID.contains("_null"))  // Grow
				task = new TaskSheet( activeGraph, targetGraph, Task::GROW, tasks.size() );
			else if (tnodeID.contains("_null")) // Shrink
				task = new TaskSheet( activeGraph, targetGraph, Task::SHRINK, tasks.size() );
			else
				task = new TaskSheet( activeGraph, targetGraph, Task::MORPH, tasks.size() );
		}

		task->setNode( snodeID );
		tasks.push_back( task );
	}
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

	this->startAllSameTime();

	// Order and group here:
	this->order();

	// Time-line slider
	if(!slider)
	{
		slider = new TimelineSlider;
		slider->reset();
		this->connect( slider, SIGNAL(timeChanged(int)), SLOT(timeChanged(int)), Qt::QueuedConnection );
		this->addItem( slider );
	}
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

		int futureStart = curStart;

		// Sort tasks by priority
		curTasks = sortTasksByPriority( curTasks );

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
			curTasks = sortTasksAsLayers( curTasks, curStart );
			foreach(Task* t, curTasks) futureStart = qMax(futureStart, t->endTime());
		}

		// Group events in same group
		Structure::Graph * g = (i == Task::SHRINK) ? activeGraph : targetGraph;
		groupStart(g, curTasks, curStart, futureStart);		

		curStart = futureStart;
	}

	// Remove large empty spaces between tasks [inefficient?]
	this->trimTasks();

	// Add small spaces between tasks
	{
		int timeSpacing = totalExecutionTime() * timeStep + 1;

		QVector<Task*> allTasks = tasksSortedByStart();

		int N = allTasks.size();
		for(int i = 0; i < N; i++)
		{
			Task * currTask = allTasks[i];

			QList<Task*> before, after;
			splitTasksStartTime(currTask->endTime() - 1, before, after);

			foreach(Task* t, after)
				t->setStart( t->start + timeSpacing );

			while(i < N-1 && allTasks[i+1]->start == currTask->start) i++;
		}
	}

	// To-do: Collect tasks together?
}

void Scheduler::trimTasks()
{
	int curTime = 0;
	forever{
		QList<Task*> before, after;
		splitTasksStartTime(curTime, before, after);

		if(after.empty()) break;

		if(!before.empty())
		{
			int end = endOf( before );
			int start = startOf( after );

			int delta = end - start;
			if(delta < 0) 
				slideTasksTime(after, delta);
		}
		curTime += 50;
	}
}

void Scheduler::groupStart( Structure::Graph * g, QList<Task*> curTasks, int curStart, int & futureStart )
{
	if(!curTasks.size()) return;

	int i = curTasks.front()->type;

	foreach(QVector<QString> group, g->groups)
	{
		QVector<Task*> tasksInGroup;
		QVector<QString> groupTarget;

		// Check which tasks are in a group
		foreach(Task * t, curTasks){
			Structure::Node * n = (i == Task::SHRINK) ? t->node() : t->targetNode();

			if(group.contains(n->id))
				tasksInGroup.push_back(t);
		}

		// Check grouping from target graph
		if(!tasksInGroup.isEmpty()){
			Structure::Graph * tg = (i == Task::SHRINK) ? tasksInGroup.front()->target : tasksInGroup.front()->active;
			groupTarget = tg->groupsOf( tasksInGroup.front()->node()->id ).front();

			if(!groupTarget.isEmpty()){
				foreach(Task * t, curTasks){
					Structure::Node * n = (i == Task::SHRINK) ? t->targetNode() : t->node();
					if(groupTarget.contains(n->id) && !tasksInGroup.contains(t))
						tasksInGroup.push_back(t);
				}
			}
		}

		curStart = futureStart;

		// Assign same start for them				
		foreach(Task * t, tasksInGroup){
			curStart = qMin(curStart, t->start);		
		}
		foreach(Task * t, tasksInGroup){
			t->setStart(curStart);
			futureStart = qMax(futureStart, t->endTime());

			t->property["grouped"] = true;
		}
	}
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

QList<Task*> Scheduler::sortTasksAsLayers( QList<Task*> currentTasks, int startTime )
{
	QList<Task*> sorted;

	QVector< QList<Task*> > groups = TaskGroups::split(currentTasks, activeGraph);

	int futureStart = -1;

	foreach(QList<Task*> group, groups)
	{
		TaskGroups::Graph g( group, activeGraph );

		QVector< QList<Task*> > layers = g.peel();

		if(currentTasks.front()->type == Task::SHRINK)
			layers = reversed(layers);

		foreach(QList<Task*> layer, layers)
		{
			foreach(Task* t, layer){
				t->setStart(startTime);
				futureStart = qMax(futureStart, t->endTime());
			}

			startTime = futureStart;

			sorted += layer;
		}
	}

	return sorted;
}

void Scheduler::reset()
{
	// Save assigned schedule
	QMap< QString,QPair<int,int> > curSchedule = getSchedule();

	// Clean previous outputs
	qDeleteAll(allGraphs);
	allGraphs.clear();

	// Clean old tasks
	foreach(Task * task, tasks)	this->removeItem(task);
	tasks.clear();

	// Remove any tags
	property.remove("timeTags");

	/// Reload graphs:	
	this->generateTasks();
	this->schedule();

	// Reassign schedule
	this->setSchedule( curSchedule );
	emit( progressChanged(0) );
	emit( hasReset() );
}

void Scheduler::executeAll()
{
	int totalTime = totalExecutionTime();
	QVector<Task*> allTasks = tasksSortedByStart();

	if( isApplyChangesUI ) qApp->setOverrideCursor(Qt::WaitCursor);

	// pre-execute
	{
		property["progressDone"] = false;
		isForceStop = false;

		emit( progressStarted() );

		// Tag interesting topology changes
		{
			property.remove("timeTags");

			QSet<int> tags;
			foreach(Task * task, allTasks){
				int time = task->start + (0.5 * task->length);
				tags.insert( time );
			}

			property["timeTags"].setValue( tags );
		}
	}

	Relink linker(this);

	// Initial setup
	{
		// Zero the geometry for null nodes
		foreach(Structure::Node * snode, activeGraph->nodes)
		{
			QString sid = snode->id;
			if (!sid.contains("_null")) continue;
			snode->setControlPoints( Array1D_Vector3(snode->numCtrlPnts(), Vector3(0,0,0)) );
			snode->property["zeroGeometry"] = true;

			bool isPartOfCutGroup = activeGraph->isInCutGroup(sid);
			bool isCutNode = activeGraph->isCutNode(sid);

			if( !isCutNode && !isPartOfCutGroup )
			{
				// Make sure it is "effectively" linked on only one existing node
				QMap<Link *,int> existValence;
				foreach(Link * edge, activeGraph->getEdges(sid)){
					QString otherID = edge->otherNode(sid)->id;
					if(otherID.contains("_null")) continue;
					existValence[edge] = activeGraph->valence(activeGraph->getNode(otherID));
				}

				// Ignore if connects to one existing
				if(existValence.size() < 2) 
					continue;

				// Pick existing node with most valence
				QList< QPair<int, Link *> > sorted = sortQMapByValue(existValence);
				Link * linkKeep = sorted.back().second;

				// Replace all edges of null into the kept edge
				foreach(Link * edge, activeGraph->getEdges(sid))
				{
					if(edge == linkKeep) continue;

					// Keep track of original edge info
					QString oldNode = edge->otherNode(sid)->id;
					edge->pushState();
					edge->replace(oldNode, linkKeep->otherNode(sid), linkKeep->getCoordOther(sid));
				}

				snode->property["edgesModified"] = true;
			}
		}

		// Relink once to place null nodes at initial positions:
		QVector<QString> aTs; 
		foreach (Task* task, allTasks) {if ( task->type != Task::GROW)	aTs << task->nodeID;}
		if (aTs.isEmpty()) aTs << allTasks.front()->nodeID;
		activeGraph->property["activeTasks"].setValue( aTs );

		linker.execute();

		// Debug: 
		//allGraphs.push_back( new Structure::Graph( *activeGraph ) );
	}

	// Execute all tasks
	for(double globalTime = globalStart; globalTime <= (globalEnd + timeStep); globalTime += timeStep)
	{
		QElapsedTimer timer; timer.start();

		// DEBUG - per pass
		activeGraph->clearDebug();

		// active tasks
		QVector<QString> aTs = activeTasks(globalTime * totalTime);
		activeGraph->property["activeTasks"].setValue( aTs );
		activeGraph->property["t"] = globalTime;

		// For visualization
		activeGraph->setPropertyAll("isActive", false);

		// Blend deltas
		blendDeltas( globalTime, timeStep );

		/// Prepare and execute current tasks
		for(int i = 0; i < (int)allTasks.size(); i++)
		{
			Task * task = allTasks[i];
			double localTime = task->localT( globalTime * totalTime );
			if( localTime < 0 || task->isDone ) continue;

			// Prepare task for grow, shrink, morph
			task->prepare();

			// Execute current task at current time
			task->execute( localTime );

			// For visualization
			if(localTime >= 0.0 && localTime < 1.0) task->node()->property["isActive"] = true;
		}

		/// Geometry morphing
		foreach(Task * task, allTasks)
		{
			double localTime = task->localT( globalTime * totalTime );
			if( localTime < 0 ) continue;
			task->geometryMorph( localTime );
		}

		/// Apply relinking
		linker.execute();

		// Output current active graph:
		allGraphs.push_back(  new Structure::Graph( *activeGraph )  );

		// DEBUG:
		activeGraph->clearDebug();

		// UI - progress visual indicator:
		int percent = globalTime * 100;
		property["progress"] = percent;
		emit( progressChanged(percent) );

		if( isForceStop ) break;
	}

	finalize();

	property["progressDone"] = true;

	if( isApplyChangesUI ) 
	{
		slider->enable();

		qApp->restoreOverrideCursor();
		QCursor::setPos(QCursor::pos());
	}

	emit( progressDone() );
}

void Scheduler::finalize()
{
	double sumDistortion = 0;

	double distThreshold = activeGraph->bbox().diagonal().norm() * 0.1;

	foreach(Node * n, activeGraph->nodes){
		if(n->property["shrunk"].toBool()) continue;

		Node * tn = targetGraph->getNode( n->property["correspond"].toString() );
		double curDistortion = n->distortion( tn );
		sumDistortion += curDistortion;
	}

	// Make sure we exactly get the target, no constraints here
	if(sumDistortion > distThreshold){
		double finalizeDuration = double(Task::DEFAULT_LENGTH) / totalExecutionTime();

		// Reassign 't' values for generated graphs
		double stretch = (totalExecutionTime() - overTime) / totalExecutionTime();
		for(int i = 0; i < (int)allGraphs.size(); i++)
			allGraphs[i]->property["t"] = stretch * (allGraphs[i]->property["t"].toDouble());

		QMap<Node*, Array1D_Vector3> curGeometry;
		foreach(Node * n, activeGraph->nodes)
			curGeometry[n] = n->controlPoints();

		int steps = int(finalizeDuration / timeStep);

		for(int u = 0; u <= steps; u++){
			double t = double(u) / steps;

			// Morph nodes
			foreach(Node * n, activeGraph->nodes){
				Node * tn = targetGraph->getNode( n->property["correspond"].toString() );
				Array1D_Vector3 finalGeometry = tn->controlPoints();
				Array1D_Vector3 newGeometry;

				for(int i = 0; i < (int) finalGeometry.size(); i++)
					newGeometry.push_back( AlphaBlend(t, curGeometry[n][i], Vector3(finalGeometry[i])) );

				n->setControlPoints( newGeometry );
			}

			allGraphs.push_back(  new Structure::Graph( *activeGraph )  );
		}

		overTime = Task::DEFAULT_LENGTH;

		// Move last tag to overtime
		QSet<int> modfiedTags = property["timeTags"].value< QSet<int> >();
		int lastVal = modfiedTags.values().back();
		modfiedTags.remove(lastVal);
		modfiedTags.insert(totalExecutionTime() - (0.5 * Task::DEFAULT_LENGTH));
		property["timeTags"].setValue(modfiedTags);
	}
}

bool Scheduler::isPartOfGrowingBranch( Task* t )
{
	return (t->type == Task::GROW) && !(t->node()->property.contains("isCutGroup"));
}

QVector<Task*> Scheduler::getEntireBranch( Task * t )
{
	QVector<Task*> branch;
	foreach(Structure::Node * n, activeGraph->nodesWithProperty("nullSet", t->node()->property["nullSet"]))
		branch.push_back( getTaskFromNodeID(n->id) );
	return branch;
}

void Scheduler::drawDebug()
{
	foreach(Task * t, tasks)
	{
		t->drawDebug();
	}
}

int Scheduler::totalExecutionTime()
{
	int endTime = 0;

	foreach(Task * t, tasks)
		endTime = qMax(endTime, t->endTime());

	return endTime + overTime;
}

void Scheduler::timeChanged( int newTime )
{
	if(!allGraphs.size()) return;

	int idx = allGraphs.size() * (double(newTime) / totalExecutionTime());

	idx = qRanged(0, idx, allGraphs.size() - 1);
	allGraphs[idx]->property["index"] = idx;

	emit( activeGraphChanged(allGraphs[idx]) );
}

void Scheduler::doBlend()
{
	foreach(Task * t, tasks) t->setSelected(false);

	// If we are re-executing, we need to reset everything
	if(allGraphs.size())
		reset();

	/// Execute the tasks on a new thread
	QtConcurrent::run( this, &Scheduler::executeAll ); // scheduler->executeAll();
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
	if(selectedItems().isEmpty())
	{
		foreach(Task * t, tasks)
			t->setX(0);
	}
	else
	{
		// Get minimum start
		int minStart = INT_MAX;
		foreach(Task * t, tasks){
			if(t->isSelected())
				minStart = qMin( minStart, t->start );
		}

		foreach(Task * t, tasks){
			if(t->isSelected())
				t->setX( minStart );
		}
	}
}

void Scheduler::startDiffTime()
{
	if(selectedItems().isEmpty())
	{
		for(int i = 0; i < (int)tasks.size(); i++){
			int startTime = 0;
			if(i > 0) startTime = tasks[i - 1]->endTime();
			tasks[i]->setX( startTime );
		}
	}
	else
	{
		int startTime = -1;
		foreach(Task * t, tasks){
			if(t->isSelected())
			{
				if(startTime < 0) startTime = t->start;
				t->setX( startTime );
				startTime += t->length;
			}
		}
	}
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

		bool isActive = task->isActive( localTime );

		// Consider future growing cut nodes as active
		bool isUngrownCut = (!task->isDone) && (task->type == Task::GROW) && (task->node()->property.contains("isCutGroup"));

		// HH - Why?
		// Consider dead links as active tasks
		//bool isDeadLink = task->isDone && (task->type == Task::SHRINK);
		bool isDeadLink = false;

		if ( isActive || isUngrownCut || isDeadLink )
		{
			aTs.push_back( task->node()->id );
		}
	}

	return aTs;
}

void Scheduler::splitTasksStartTime( int startTime, QList<Task*> & before, QList<Task*> & after )
{
	foreach(Task * t, tasks){
		if(t->start < startTime)
			before.push_back(t);
		else
			after.push_back(t);
	}
}

void Scheduler::slideTasksTime( QList<Task*> list_tasks, int delta )
{
	foreach(Task * t, list_tasks){
		t->setStart( t->start + delta );
	}
}

int Scheduler::startOf( QList<Task*> list_tasks )
{
	int start = INT_MAX;
	foreach(Task * t, list_tasks) start = qMin(start, t->start);
	return start;
}

int Scheduler::endOf( QList<Task*> list_tasks )
{
	int end = -INT_MAX;
	foreach(Task * t, list_tasks) end = qMax(end, t->endTime());
	return end;
}

void Scheduler::addMorphTask( QString nodeID )
{
	Task * prev = tasks.back();

	Task * task;

	if(activeGraph->getNode(nodeID)->type() == Structure::CURVE)
		task = new TaskCurve( activeGraph, targetGraph, Task::MORPH, tasks.size() );

	if(activeGraph->getNode(nodeID)->type() == Structure::SHEET)
		task = new TaskSheet( activeGraph, targetGraph, Task::MORPH, tasks.size() );

	task->setNode( nodeID );
	tasks.push_back( task );

	// Placement
	task->moveBy(prev->x() + prev->width,prev->y() + (prev->height));
	task->currentTime = task->x();
	task->start = task->x();

	// Add to scene
	this->addItem( task );
}

void Scheduler::blendDeltas( double globalTime, double timeStep )
{
	if (globalTime >= 1.0) return;

	Q_UNUSED( timeStep );
	//double alpha = timeStep / (1 - globalTime);

	foreach(Structure::Link* l, activeGraph->edges)
	{
		Structure::Link* tl = targetGraph->getEdge(l->property["correspond"].toInt());
		if (!tl) continue;

		//Alpha value:
		double alpha = 0;
		{
			Task * sTask1 = l->n1->property["task"].value<Task*>();
			Task * sTask2 = l->n2->property["task"].value<Task*>();

			if(sTask1->isDone && sTask2->isDone) 
			{
				alpha = 1.0;
			}
			else if( sTask1->isDone )
			{
				alpha = sTask2->property["t"].toDouble();
			}
			else
			{
				alpha = sTask1->property["t"].toDouble();
			}
		}

		Vector3d sDelta = l->delta();
		Vector3d tDelta = tl->property["delta"].value<Vector3d>();

		// flip tDelta if is not consistent with sDeltas
		Node *sn1 = l->n1;

		Vector3d blendedDelta = AlphaBlend(alpha, sDelta, tDelta);
		l->property["blendedDelta"].setValue( blendedDelta );

		// Visualization
		activeGraph->vs3.addVector(l->position(sn1->id), blendedDelta);
		//activeGraph->vs.addVector(l->position(sn1->id), tDelta);
	}
}

void Scheduler::setGDResolution( double r)
{
	DIST_RESOLUTION = r;
}

void Scheduler::setTimeStep( double dt )
{
	timeStep = dt;
}

void Scheduler::loadSchedule(QString filename)
{
	QFile file(filename);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
	QFileInfo fileInfo(file.fileName());

	QTextStream in(&file);

	int N = 0; 
	QString nodeID;
	int start = 0, length = 0;

	in >> N; if(N != tasks.size()) { qDebug() << "Invalid schedule!"; return; }

	for(int i = 0; i < N; i++)
	{
		in >> nodeID >> start >> length;
		Task * t = getTaskFromNodeID( nodeID );

		if( t )
		{
			t->setStart( start );
			t->setLength( length );
		}
	}

	file.close();
}

void Scheduler::saveSchedule(QString filename)
{
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QFileInfo fileInfo(file.fileName());

	QTextStream out(&file);
	out << tasks.size() << "\n";
	foreach(Task * t, tasks)	out << t->nodeID << " " << t->start << " " << t->length << "\n";
	file.close();
}

ScheduleType Scheduler::getSchedule()
{
	ScheduleType result;
	foreach(Task * t, tasks) result[t->nodeID] = qMakePair( t->start, t->length );
	return result;
}

ScheduleType Scheduler::reversedSchedule(const ScheduleType & fromSchedule)
{
	ScheduleType result;

	QMap< int, QVector<QString> > startTask;

	foreach(QString task, fromSchedule.keys())
		startTask[fromSchedule[task].first].push_back(task);

	// These keys are sorted in increasing order
	QVector<int> originalTimes = startTask.keys().toVector();

	// Shuffle them
	QVector<int> startTimes = originalTimes;
	std::reverse(startTimes.begin(), startTimes.end());

	for(int i = 0; i < (int)startTimes.size(); i++)
	{
		int oldStart = originalTimes[i];
		int newStart = startTimes[i];

		QVector<QString> curTasks = startTask[oldStart];

		foreach(QString t, curTasks)
		{
			result[t] = qMakePair(newStart, fromSchedule[t].second);
		}
	}

	return result;
}

void Scheduler::setSchedule( ScheduleType fromSchedule )
{
	Task * t = NULL;
	foreach(QString nodeID, fromSchedule.keys())
	{
		if(t = getTaskFromNodeID(nodeID))
		{
			t->setStart(fromSchedule[nodeID].first);
			t->setLength(fromSchedule[nodeID].second);
		}
	}
}

void Scheduler::defaultSchedule()
{
	this->startDiffTime();
	this->order();
}

void Scheduler::mouseReleaseEvent( QGraphicsSceneMouseEvent * event )
{
	emit( updateExternalViewer() );
	QGraphicsScene::mouseReleaseEvent(event);
}

void Scheduler::mousePressEvent( QGraphicsSceneMouseEvent * event )
{
	property["mouseFirstPress"] = true;

	QGraphicsScene::mousePressEvent(event);

	// Reset on changes to schedule
	if(allGraphs.size()){
		foreach(Task* t, tasks){
			if(t->isSelected()){
				this->reset();
				emitUpdateExternalViewer();
			}
		}
	}
}

void Scheduler::mouseMoveEvent( QGraphicsSceneMouseEvent * event )
{
	if(property["mouseFirstPress"].toBool()){
		property["mouseFirstPress"] = false;
		emitUpdateExternalViewer();
	}

	QGraphicsScene::mouseMoveEvent(event);
}

void Scheduler::emitUpdateExternalViewer()
{
	emit( updateExternalViewer() );
}

void Scheduler::emitProgressStarted()
{
	emit( progressStarted() );
}

void Scheduler::emitProgressChanged( int val )
{
	emit( progressChanged(val) );
}

void Scheduler::emitProgressedDone()
{
	emit( progressDone() );
}

void Scheduler::shuffleSchedule()
{
	if(allGraphs.size()) reset();

	QMap< int, QVector<Task*> > startTask;

	foreach(Task * t, this->tasks)
		startTask[t->start].push_back(t);

	// These keys are sorted in increasing order
	QVector<int> originalTimes = startTask.keys().toVector();

	// Shuffle them
	QVector<int> startTimes = originalTimes;
	std::random_shuffle(startTimes.begin(), startTimes.end());

	for(int i = 0; i < (int)startTimes.size(); i++)
	{
		int oldStart = originalTimes[i];
		int newStart = startTimes[i];

		QVector<Task*> curTasks = startTask[oldStart];

		foreach(Task * t, curTasks)
		{
			t->setStart( newStart );
		}
	}
}

QVector<ScheduleType> Scheduler::manyRandomSchedules(int N)
{
	QVector<ScheduleType> schedules;

	if(allGraphs.size()) reset();
	QMap< int, QVector<Task*> > startTask;
	foreach(Task * t, this->tasks)	startTask[t->start].push_back(t);
	QVector<int> originalTimes = startTask.keys().toVector();
	QVector<int> startTimes = originalTimes;

	// Add the default scheduling as first possibility
	schedules.push_back( getSchedule() );

	// Add the reverse of the default scheduling
	if(N > 1) schedules.push_back( Scheduler::reversedSchedule(schedules.front()) );

	for(int itr = 2; schedules.size() < N; itr++)
	{
		std::random_shuffle(startTimes.begin(), startTimes.end());

		ScheduleType s;

		for(int i = 0; i < (int)startTimes.size(); i++)
		{
			int oldStart = originalTimes[i];
			int newStart = startTimes[i];

			QVector<Task*> curTasks = startTask[oldStart];

			foreach(Task * t, curTasks)
				s[ t->nodeID ] = qMakePair(newStart, t->length);
		}

		schedules.push_back( s );
	}

	return schedules;
}

QVector<ScheduleType> Scheduler::allSchedules()
{
	QVector<ScheduleType> schedules;

	if(allGraphs.size()) reset();
	QMap< int, QVector<Task*> > startTask;
	foreach(Task * t, this->tasks)	startTask[t->start].push_back(t);
	QVector<int> originalTimes = startTask.keys().toVector();
	QVector<int> startTimes = originalTimes;

	do {
		ScheduleType s;

		for(int i = 0; i < (int)startTimes.size(); i++){
			int oldStart = originalTimes[i];
			int newStart = startTimes[i];
			QVector<Task*> curTasks = startTask[oldStart];

			foreach(Task * t, curTasks)
				s[ t->nodeID ] = qMakePair(newStart, t->length);
		}

		schedules.push_back( s );
	} while( std::next_permutation(startTimes.begin(), startTimes.end()) );

	return schedules;
}

QVector<Structure::Graph*> Scheduler::interestingInBetweens(int N)
{
	QVector<Structure::Graph*> result;
	if(!allGraphs.size()) return result;

	QSet<int> tags = property["timeTags"].value< QSet<int> >();
	int totalTime = totalExecutionTime();

	QVector<double> times;

	foreach(int tag, tags){
		double t = double(tag) / totalTime;
		times.push_back(t);
	}
	qSort(times);

	// Missing some more samples
	while(times.size() < N)
	{
		QMap<double, int> intervals;
		for(int i = 0; i + 1 < times.size(); i++)
			intervals[times[i+1] - times[i]] = i;

		int selected = intervals[intervals.keys().back()];
		double length = times[ selected + 1 ] - times[ selected ];
		double midTime = (length * 0.5) + times[selected];

		times.push_back(midTime);
		qSort(times);
	}

	// Having a lot more samples
	while(times.size() > N)
	{
		QMap<double, int> intervals;

		// Note we start from '1'
		for(int i = 1; i + 1 < times.size() - 1; i++)
			intervals[times[i+1] - times[i]] = i;

		int selected = intervals[intervals.keys().front()];
		times.remove( selected );
	}

	foreach(double t, times)
		result.push_back( allGraphs[ t * (allGraphs.size() - 1) ] );

	return result;
}
