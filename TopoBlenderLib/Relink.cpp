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
	propagationQueue.clear();
	foreach (Task* t, s->tasks) 
	{
		t->property["relinked"] = false;
		t->property["propagated"] = false;
	}

	// Set up source nodes
	QVector<QString> activeNodeIDs = activeGraph->property["activeTasks"].value< QVector<QString> >();
	if( activeNodeIDs.isEmpty() ) return;

	Task* currTask = s->getTaskFromNodeID(activeNodeIDs.front());
	if (currTask->type != Task::MORPH) return;
	if (currTask->property["isCrossing"].toBool()) return;
	
	// All active tasks are sources
	foreach (QString nID, activeNodeIDs)
	{
		Task* task = s->getTaskFromNodeID(nID);
		propagationQueue.enqueue(task);
		task->property["propagated"] = true;
	}

	// propagating
	while(!propagationQueue.isEmpty())
	{
		Task* nextTask = propagationQueue.dequeue();

		// Fix nextTask according to constraints
		relinkTask(nextTask, globalTime);

		// Propagate through nextTask
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

		// Skip shrunk and non-grown nodes
		if (otherTask->node()->property["shrunk"].toBool()) continue;
		if (otherTask->type == Task::GROW && !otherTask->isDone) continue;

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

		// Skip shrunk and non-grown nodes
		if (otherTask->node()->property["shrunk"].toBool()) continue;
		if (otherTask->type == Task::GROW && !otherTask->isDone) continue;

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

	QVector<LinkConstraint> consts;
	if (constraints.contains(otherTask)) consts = constraints[otherTask];

	int N = consts.size();

	// Translate done tasks
	if (otherTask->isDone && N > 0)
	{
		// Use all constraints
		Vec3d translation(0);
		foreach(LinkConstraint c, consts)
		{
			Structure::Link * link = c.link;

			Vec3d oldPos = link->position(other->id);
			Vec3d linkPosOther = link->positionOther(other->id);
			Vec3d delta = getDelta(link, other->id);
			Vector3 newPos = linkPosOther + delta;

			translation += newPos - oldPos;
		}
		other->moveBy(translation / N);
		
		// Visualization
		foreach(LinkConstraint c, consts)
		{
			Structure::Link* link = c.link;
			Vector3 linkPosOther = link->positionOther(other->id);
			Vec3d delta = getDelta(link, other->id);

			activeGraph->vs2.addVector( linkPosOther, delta );
		}

	}
	// Translate future tasks if only one constraint
	else if( N == 1 )
	{
		LinkConstraint constraint = consts.front();
		Structure::Link * link = constraint.link;

		Vec3d oldPos = link->position(other->id);
		Vec3d linkPosOther = link->positionOther(other->id);
		Vec3d delta = getDelta(link, other->id);
		Vector3 newPos = linkPosOther + delta;

		// Translate
		other->moveBy(newPos - oldPos);

		// Visualization
		activeGraph->vs2.addVector( linkPosOther, delta );
	}
	// Deform future tasks by two handles if multiple constrains exist
	else if (N > 1)
	{
		// Pickup two constrains to deform the node
		LinkConstraint cA = consts.front();
		LinkConstraint cB = consts.back();
		Structure::Link *linkA = cA.link, *linkB = cB.link;

		// Two Handles and newPos:
		Vec4d handleA = linkA->getCoord(other->id).front();
		Vector3 linkPosOtherA = linkA->positionOther(other->id);
		Vector3 deltaA = getDelta(linkA, other->id);
		Vector3 newPosA = linkPosOtherA + deltaA;

		Vec4d handleB = linkB->getCoord(other->id).front();
		Vector3 linkPosOtherB = linkB->positionOther(other->id);
		Vector3 deltaB = getDelta(linkB, other->id);
		Vector3 newPosB = linkPosOtherB + deltaB;

		// Deform two handles
		other->deformTwoHandles(handleA, newPosA, handleB, newPosB);

		// Visualization
		activeGraph->vs2.addVector( linkPosOtherA, deltaA );
		activeGraph->vs2.addVector( linkPosOtherB, deltaB );
	}

	// Visualization
	foreach(LinkConstraint c, consts)
	{
		Structure::Link* link = c.link;
		Vector3 linkPosOther = link->positionOther(other->id);
		Vec3d delta = getDelta(link, other->id);

		activeGraph->vs.addVector( linkPosOther, delta );
	}
}

Vector3 Relink::getDelta( Structure::Link * link, QString otherID )
{
	Vector3 delta = link->property["blendedDelta"].value<Vector3>();
	if(link->n1->id == otherID) delta *= -1;
	return delta;
}
