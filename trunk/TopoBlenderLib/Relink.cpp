#include "Relink.h"
#include "Scheduler.h"

Relink::Relink(Scheduler *scheduler)
{
    this->s = scheduler;
    this->activeGraph = scheduler->activeGraph;
    this->targetGraph = scheduler->targetGraph;

	checkRelinkability = true;
}

void Relink::execute()
{
	// initial
	constraints.clear();
	propagationQueue.clear();
	foreach (Task* t, s->tasks) 
	{
		t->property["relinked"] = false;
		t->property["propagated"] = false;
	}

	// Set up starting nodes
	QVector<QString> activeNodeIDs = activeGraph->property["activeTasks"].value< QVector<QString> >();
	foreach (QString nID, activeNodeIDs)
	{
		Task* task = s->getTaskFromNodeID(nID);

		// Skip all crossing nodes
		if ( doesPropagate(task) )
		{
			propagationQueue.enqueue(task);
			task->property["propagated"] = true;
		}
	}

	// propagating
	while( !propagationQueue.isEmpty() )
	{
		Task* task = propagationQueue.dequeue();

		// Fix current task according to constraints
		fixTask(task);

		// Propagate from task
		propagateFrom(task);
	}
}

void Relink::fixTask( Task* task )
{
	task->property["relinked"] = true;

	Structure::Node * n = task->node();

	QVector<LinkConstraint> consts;
	if ( constraints.contains(task) ) consts = constraints[task];
	int N = consts.size();

	// No constraints for task
	if (N == 0) return;

	// Crossing node is still fixable 
	// Translate done tasks

	bool fixedSize = false;
	if (task->type == Task::MORPH && task->isDone && !task->property["isCrossing"].toBool())
		fixedSize = true;

	if ( fixedSize && N > 0 )
	{
		// Use all constraints
		Vec3d translation(0);
		foreach(LinkConstraint c, consts)
		{
			Structure::Link * link = c.link;

			Vec3d oldPos = link->position(n->id);
			Vec3d linkPosOther = link->positionOther(n->id);
			Vec3d delta = getToDelta(link, n->id);
			Vector3 newPos = linkPosOther + delta;

			translation += newPos - oldPos;
		}
		n->moveBy( translation / N );
		
		// Visualization
		foreach(LinkConstraint c, consts)
		{
			Structure::Link* link = c.link;
			Vector3 linkPosOther = link->positionOther(n->id);
			Vec3d delta = getToDelta(link, n->id);

			activeGraph->vs2.addVector( linkPosOther, delta );
		}
	}
	else 
	{
		// Special case to relink null nodes
		if (task->ungrownNode(task->nodeID)) N = 1;
		
		// Translate future tasks if only one constraint
		if( N == 1 )
		{
			LinkConstraint constraint = consts.front();
			Structure::Link * link = constraint.link;

			Vec3d oldPos = link->position(n->id);
			Vec3d linkPosOther = link->positionOther(n->id);
			Vec3d delta = getToDelta(link, n->id);
			Vector3 newPos = linkPosOther + delta;

			// Translate
			n->moveBy(newPos - oldPos);

			// Visualization
			activeGraph->vs2.addVector( linkPosOther, delta );
		}
		// Deform future tasks by two handles if multiple constrains exist
		else if ( N > 1 )
		{
			// Pickup two constrains to deform the node
			LinkConstraint cA = consts.front();
			LinkConstraint cB = consts.back();
			Structure::Link *linkA = cA.link, *linkB = cB.link;

			// Two Handles and newPos:
			Vec4d handleA = linkA->getCoord(n->id).front();
			Vector3 linkPosOtherA = linkA->positionOther(n->id);
			Vector3 deltaA = getToDelta(linkA, n->id);
			Vector3 newPosA = linkPosOtherA + deltaA;

			Vec4d handleB = linkB->getCoord(n->id).front();
			Vector3 linkPosOtherB = linkB->positionOther(n->id);
			Vector3 deltaB = getToDelta(linkB, n->id);
			Vector3 newPosB = linkPosOtherB + deltaB;

			// In case two handles or two new positions are two close
			double handleDiff = (handleA - handleB).norm();
			double newPosDiff = (newPosA - newPosB).norm();
			if (handleDiff < 0.1 || newPosDiff < 0.05)
			{
				// Pick first one only
				Vec3d oldPos = linkA->position(n->id);
				Vec3d linkPosOther = linkA->positionOther(n->id);
				Vec3d delta = getToDelta(linkA, n->id);
				Vector3 newPos = linkPosOther + delta;

				// Translate
				n->moveBy(newPos - oldPos);
				activeGraph->vs2.addVector( linkPosOther, delta );
			}
			// Deform two handles
			else 
			{
				n->deformTwoHandles(handleA, newPosA, handleB, newPosB);

				// Visualization
				activeGraph->vs2.addVector( linkPosOtherA, deltaA );
				activeGraph->vs2.addVector( linkPosOtherB, deltaB );
			}
		}
	}

	// Visualization
	foreach(LinkConstraint c, consts)
	{
		Structure::Link* link = c.link;
		Vector3 linkPosOther = link->positionOther(n->id);
		Vec3d delta = getToDelta(link, n->id);

		activeGraph->vs.addVector( linkPosOther, delta );
	}
}

void Relink::propagateFrom( Task* task )
{
	Structure::Node * n = task->node();
	QVector<Structure::Link*> edges = activeGraph->getEdges(n->id);

	// Virtual cut node
	bool virtualCutting =  task->isCutting() && !task->isCuttingReal();

	foreach(Structure::Link* link, edges)
	{
		Structure::Node * other = link->otherNode(n->id);
		Task * otherTask = s->getTaskFromNodeID(other->id);

		// Virtual cut node
		// Don't propagate to its real neighbours
		if (virtualCutting && !task->ungrownNode(otherTask->nodeID)) continue;

		// Add to queue
		if ( !otherTask->property["propagated"].toBool() )
		{
			propagationQueue.enqueue(otherTask);
			otherTask->property["propagated"] = true;
		}

		// generate constrains to unfixed task
		if ( !otherTask->property["relinked"].toBool() )
			constraints[ otherTask ].push_back( LinkConstraint(link, task, otherTask) );
	}
}

Vector3 Relink::getToDelta( Structure::Link * link, QString toOtherID )
{
	// by default delta = n2 - n1
	Vector3 delta = link->property["blendedDelta"].value<Vector3>();
	if(link->n1->id == toOtherID) delta *= -1;
	return delta;
}

bool Relink::doesPropagate(Task* task)
{
	if (task->isCutting()) return true;
	if (task->type == Task::MORPH && !task->isCrossing()) return true;
	return false;
}
