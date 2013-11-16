#include <QStack>

#include "Relink.h"
#include "Scheduler.h"

#include "MinBall.h"

Relink::Relink(Scheduler *scheduler)
{
    this->s = scheduler;
    this->activeGraph = scheduler->activeGraph;
    this->targetGraph = scheduler->targetGraph;
}

void Relink::execute()
{
	// initial
	constraints.clear();
	foreach (Task* t, s->tasks) 
	{
		t->property["relinked"] = false;
		t->property["propagated"] = false;
	}

	// Tracking
	propagationGraph.clear();
	propagationIndex = 0;

	// Find propagation levels via BFS
	QVector< QVector<Task*> > propagationLevel;
	propagationLevel.resize(1);

	QVector<QString> activeNodeIDs = activeGraph->property["activeTasks"].value< QVector<QString> >();

	// Initial propagation level
	foreach (QString nID, activeNodeIDs){
		Task* task = s->getTaskFromNodeID(nID);
		if ( doesPropagate(task) ){
			propagationLevel[0].push_back(task);
			task->property["propagated"] = true;
		}
	}

	forever{
		QVector<Task *> curLevel;

		// Visit nodes in levels
		foreach(Task* task, propagationLevel.back())
		{
			foreach(Structure::Link* link, activeGraph->getEdges(task->nodeID))
			{
				// Self referring edges should be ignored
				if(link->n1->id == link->n2->id) continue;

				Structure::Node * other = link->otherNode(task->nodeID);
				Task * otherTask = s->getTaskFromNodeID(other->id);
				if (otherTask->property["propagated"].toBool()) continue;

				if(!curLevel.contains(otherTask)) curLevel.push_back( otherTask );

				// Add constraint
				constraints[ otherTask ].push_front( LinkConstraint(link, task, otherTask) );

				// Tracking
				propagationGraph[task->nodeID].push_back(qMakePair(otherTask->nodeID, propagationIndex++));
			}
		}

		// Mark elements in level as visited
		foreach(Task* task, propagationLevel.back())
				task->property["propagated"] = true;

		if(curLevel.isEmpty()) break;
		propagationLevel.push_back( curLevel );
	}

	// Apply constraints
	for(int i = 0; i < propagationLevel.size(); i++)
	{
		foreach(Task* task, propagationLevel[i]) 
		{
			fixTask(task);

			// Override relinking for splitting case
			if(task->node()->property["taskTypeReal"].toInt() == Task::SPLIT && !task->isReady){
				foreach(QString sibling, activeGraph->groupsOf(task->nodeID).back()){
					Task * otherTask = s->getTaskFromNodeID( sibling );
					if(otherTask->node()->property["taskTypeReal"].toInt() == Task::SPLIT && !otherTask->isReady)
						otherTask->node()->setControlPoints(task->node()->controlPoints());
				}
			}
		}
	}

	// Tracking
	if( true ){
		QStringList graph;
		graph << "digraph G{";
		foreach(QString n1, propagationGraph.keys()){
			foreach(PropagationEdge edge, propagationGraph[n1]){
				int idx = edge.second;
				double t = 1.0 - (double(idx) / (propagationIndex-1));
				int penWidth = qMax(1, int(t * 10));
				graph << QString("%1 -> %2 [penwidth=%3, color=\"0.0 1.0 %4\"];").arg(n1).arg(edge.first).arg(penWidth).arg(t);
			}
		}
		graph << "}";
		activeGraph->property["relinkGraph"] = graph.join("\n");
	}
}

void Relink::fixTask( Task* task )
{
	task->property["relinked"] = true;

	Structure::Node * n = task->node();

	QVector<LinkConstraint> consts;
	if ( constraints.contains(task) ) consts = constraints[task];
	if(!consts.size()) return;

	// Useful for debugging:
	{
		QStringList listRelinks;
		foreach(LinkConstraint c, consts) listRelinks << c.link->otherNode(n->id)->id;
		n->property["preConsts"].setValue( listRelinks );
	}

	bool fixedSize = false;
	if (task->type == Task::MORPH && task->isDone && !task->property["isCrossing"].toBool())
		fixedSize = true;

	/// Ignore constraints that will cause large distortions
	// CASE: link is moving around parts
	{
		QVector<LinkConstraint> keep;
		foreach(LinkConstraint c, consts){
			QVector< GraphDistance::PathPointPair > path = c.link->property["path"].value< QVector< GraphDistance::PathPointPair > >();
			if((path.size() && c.task->isDone) || (path.size() < 4))
				keep.push_back(c);
		}

		if(!keep.size()) 
			fixedSize = true;
		else if (keep.size() != consts.size())
			consts = keep;
	}

	// CASE: try to avoid using constraints that are not point based
	{
		QStack<int> toSwap;
		for(int i = 0; i < consts.size(); i++){
			if(consts[i].link->type == Structure::POINT_EDGE && !toSwap.isEmpty())
				std::swap( consts[i], consts[toSwap.pop()] );
			if(i < 2 && (consts[i].link->type == Structure::LINE_EDGE)) toSwap.push(i);
		}
	}

	// CASE: sheets are more rigid than curves
	{
		if(n->type() == Structure::SHEET && consts.size() > 2)
			fixedSize = true;
	}

	// CASE: ignore constraints of non-existing parts when connected to others
	{
		if(consts.size() > 1)
		{
			QVector<LinkConstraint> keep;
			foreach(LinkConstraint c, consts)
			{
				if(!c.task->isReady && c.task->type == Task::GROW) continue;
				keep.push_back(c);
			}
			if(keep.size()) consts = keep;
		}
	}

	int N = consts.size();

	// No constraints for task
	if (N == 0) return;

	n->property["fixedSize"] = fixedSize;

	// Useful for debugging:
	{
		QStringList listRelinks;
		foreach(LinkConstraint c, consts) listRelinks << c.link->otherNode(n->id)->id;
		n->property["postConsts"].setValue( listRelinks );
	}

	// Apply constraints
	if ( fixedSize && N > 0 )
	{
		std::vector<Vector3> oldPoints, newPoints;

		// Use all constraints
		foreach(LinkConstraint c, consts)
		{
			Structure::Link * link = c.link;

			Vector3d oldPos = link->position(n->id);
			Vector3d linkPosOther = link->positionOther(n->id);
			Vector3d delta = getToDelta(link, n->id);
			Vector3 newPos = linkPosOther + delta;

			oldPoints.push_back(oldPos);
			newPoints.push_back(newPos);
		}

		Vector3 translation = QuickMinBall<Vector3>(newPoints).center - QuickMinBall<Vector3>(oldPoints).center;

		n->moveBy( translation );

		// Debug:
		n->property["fixedTranslation"].setValue( translation );
		
		// Visualization
		foreach(LinkConstraint c, consts)
		{
			Structure::Link* link = c.link;
			Vector3 linkPosOther = link->positionOther(n->id);
			Vector3d delta = getToDelta(link, n->id);

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

			Vector3d oldPos = link->position(n->id);
			Vector3d linkPosOther = link->positionOther(n->id);
			Vector3d delta = getToDelta(link, n->id);
			Vector3 newPos = linkPosOther + delta;

			// Translate
			n->moveBy(newPos - oldPos);

			// Visualization
			activeGraph->vs2.addVector( linkPosOther, delta );
		}
		// Deform future tasks by two handles if multiple constrains exist
		else if ( N > 1 )
		{
			/// Pickup two constrains to deform the node:
			LinkConstraint cA = consts[0];
			LinkConstraint cB = consts[1];
			Structure::Link *linkA = cA.link, *linkB = cB.link;

			// Two Handles and newPos:
			Vector4d handleA = linkA->getCoord(n->id).front();
			Vector3 linkPosOtherA = linkA->positionOther(n->id);
			Vector3 deltaA = getToDelta(linkA, n->id);
			Vector3 newPosA = linkPosOtherA + deltaA;

			Vector4d handleB = linkB->getCoord(n->id).front();
			Vector3 linkPosOtherB = linkB->positionOther(n->id);
			Vector3 deltaB = getToDelta(linkB, n->id);
			Vector3 newPosB = linkPosOtherB + deltaB;

			// In case two handles or two new positions are two close
			double handleDiff = (handleA - handleB).norm();
			double newPosDiff = (newPosA - newPosB).norm();
			if (newPosDiff < 0.01 || handleDiff < 0.25)
			{
				// Pick first one only
				Vector3d oldPos = linkA->position(n->id);
				Vector3d linkPosOther = linkA->positionOther(n->id);
				Vector3d delta = getToDelta(linkA, n->id);
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
	if (task->isCutting(true)) return true;
	if (task->type == Task::MORPH && !task->isCrossing()) return true;
	return false;
}
