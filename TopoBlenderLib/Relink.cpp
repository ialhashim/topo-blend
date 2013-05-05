#include "Relink.h"
#include "Scheduler.h"

Relink::Relink(Scheduler *scheduler)
{
    this->s = scheduler;
    this->activeGraph = scheduler->activeGraph;
    this->targetGraph = scheduler->targetGraph;
}

void Relink::execute(int globalTime)
{
	// initial
	constraints.clear();
	foreach (Task* t, s->tasks) 
	{
		t->property["relinked"] = false;
		t->property["propagated"] = false;
	}

	// active tasks are sources of propagation
	QVector<QString> activeNodeIDs = activeGraph->property["activeTasks"].value< QVector<QString> >();
	foreach (QString nID, activeNodeIDs)
	{
		Task* task = s->getTaskFromNodeID(nID);

		// relink only when there are links are broken
		if (task->type != Task::MORPH ) return;

		propagationQueue.enqueue(task);
		task->property["propagated"] = true;
	}

	// propagating
	while(!propagationQueue.isEmpty())
	{
		Task* nextTask = propagationQueue.dequeue();

		relinkTask(nextTask, globalTime);

		createConstraintsFromTask(nextTask);
		propagateFrom(nextTask);
	}
}

void Relink::createConstraintsFromTask( Task* task )
{
	Structure::Node * n = task->node();
    QVector<Structure::Link*> edges = activeGraph->getEdges(n->id);

    foreach(Structure::Link* link, edges)
    {
        Structure::Node * other = link->otherNode(n->id);
        Task * otherTask = s->getTaskFromNodeID(other->id);

		// skip relinked task
		if (otherTask->property["relinked"].toBool()) continue;
        
        constraints[ otherTask ].push_back( LinkConstraint(link, task, otherTask) );
    }
}

void Relink::propagateFrom( Task* task )
{
	Structure::Node * n = task->node();
	QVector<Structure::Link*> edges = activeGraph->getEdges(n->id);

	foreach(Structure::Link* link, edges)
	{
		Structure::Node * other = link->otherNode(n->id);
		Task * otherTask = s->getTaskFromNodeID(other->id);

		// skip propagated task
		if (otherTask->property["propagated"].toBool()) continue;

		propagationQueue.enqueue(otherTask);
		otherTask->property["propagated"] = true;
	}
}


void Relink::relinkTask( Task* otherTask, int globalTime )
{
	otherTask->property["relinked"] = true;

	Structure::Node * other = otherTask->node();

	QVector<LinkConstraint> consts = constraints[otherTask];
	int N = consts.size();

	if( N == 1 )
	{
		LinkConstraint constraint = consts.front();

		Task * task = constraint.task;
		Structure::Link * link = constraint.link;
		Structure::Node * n = task->node();

		// Check for changing end edges
		if( !link->hasNode(n->id) || !link->hasNode(other->id) )	return;

		Vec4d handle = link->getCoordOther(n->id).front();
		Vector3 linkPos = link->position(n->id);

		Vector3 delta = link->property["delta"].value<Vector3>();
		if(link->n1->id == other->id) delta *= -1;

		Vector3 newPos = linkPos + delta;

		// translate by moving handle
		other->deformTo( handle, newPos, true );

		// Visualization
		activeGraph->vs.addVector( linkPos, delta );
	}

	if( N > 1 )
	{
		// Pickup two constrains to deform the node
		LinkConstraint cA = consts.front();
		LinkConstraint cB = consts.back();

		// Two Handles:
		Task *taskA = cA.task, *taskB = cB.task;
		Structure::Link *linkA = cA.link, *linkB = cB.link;

		Vec4d handleA = linkA->getCoord(other->id).front();
		Vector3 linkPosA = linkA->positionOther(other->id);
		Vector3 deltaA = linkA->property["delta"].value<Vector3>();
		if(linkA->n1->id == other->id) deltaA *= -1;
		Vector3 newPosA = linkPosA + deltaA;

		Vec4d handleB = linkB->getCoord(other->id).front();
		Vector3 linkPosB = linkB->positionOther(other->id);
		Vector3 deltaB = linkB->property["delta"].value<Vector3>();
		if(linkB->n1->id == other->id) deltaB *= -1;
		Vector3 newPosB = linkPosB + deltaB;

		other->deformTwoHandles(handleA, newPosA, handleB, newPosB);

		// Visualization
		activeGraph->vs.addVector( linkPosA, deltaA );
		activeGraph->vs.addVector( linkPosB, deltaB );
	}
}
