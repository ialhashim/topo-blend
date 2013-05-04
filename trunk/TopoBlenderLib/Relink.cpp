#include "Relink.h"
#include "Scheduler.h"

Relink::Relink(Scheduler *scheduler)
{
    this->s = scheduler;
    this->activeGraph = scheduler->activeGraph;
    this->targetGraph = scheduler->targetGraph;
}

void Relink::prepare( Task *task )
{
    Structure::Node * n = task->node();
    QVector<Structure::Link*> edges = activeGraph->getEdges(n->id);

    foreach(Structure::Link* link, edges)
    {
        Structure::Node * other = link->otherNode(n->id);
        Task * otherTask = s->getTaskFromNodeID(other->id);

		bool isOtherTaskDone = otherTask->isDone;
		bool isOtherTaskGrow = (otherTask->type == Task::GROW);
		bool isOtherCutGroup = other->property.contains("isCutGroup");

        if( isOtherTaskDone && !isOtherCutGroup ) continue;
        
        Vec3d delta = link->positionOther(n->id) - link->position(n->id);

        // Threshold for zero change
        if(delta.norm() < 1e-7) delta = Vector3(0);

        // Delta should not be larger than expected
        /*Structure::Link * tlink = targetGraph->getEdge( link->property["correspond"].toString() );
        QString tnodeID = n->property["correspond"].toString();
        if(tlink->hasNode(tnodeID)){
            Vec3d deltaTarget = tlink->positionOther(tnodeID) - tlink->position(tnodeID);
            if(delta.norm() > deltaTarget.norm())
                continue;
        }*/

        // Skip for shrinking non-cuts
        if( task->type == Task::SHRINK && !n->property.contains("isCutGroup") )
            continue;

        // Check if relinking not grown branch
        if( s->isPartOfGrowingBranch( otherTask ) )
            continue;

        constraints[ otherTask ].push_back( LinkConstraint(delta, link, task, otherTask) );
    }
}

void Relink::relink( int globalTime )
{
    foreach(Task * otherTask, constraints.keys())
    {
        Structure::Node * other = otherTask->node();

		//double hdistBefore = HausdorffDistance( other->controlPoints(), tother->controlPoints() );
		//Array1D_Vector3 cptsBefore = other->controlPoints();

        QVector<LinkConstraint> consts = constraints[otherTask];
        int N = consts.size();

        if( N == 1 )
        {
            LinkConstraint constraint = consts.front();

            Task * task = constraint.task;
            Structure::Link * link = constraint.link;
            Structure::Node * n = task->node();

            // Check for changing end edges
            if( !link->hasNode(n->id) || !link->hasNode(other->id) )	continue;

            Vec4d handle = link->getCoordOther(n->id).front();

            Vector3 linkPosOther = link->positionOther(n->id);
            Vector3 linkPos = link->position(n->id);
            Vector3 newPos = linkPos + constraint.delta;

            //bool isRigid = otherTask->isDone;
			bool isRigid = true;

            //if(other->type() == Structure::CURVE){
                //if( abs(handle[0] - 0.5) < 0.1 )
                    //isRigid = true;
            //}

            other->deformTo( handle, newPos, isRigid );

            activeGraph->vs.addVector( linkPos, constraint.delta );
        }

        if( N > 1 )
        {
            Vector3 delta(0);
            int M = 0;

            foreach( LinkConstraint constraint, consts )
            {
                Task * task = constraint.task;
                Structure::Link * link = constraint.link;
                Structure::Node * n = task->node();

                // Check for changing end edges
                if( !link->hasNode(n->id) || !link->hasNode(other->id) )	continue;

                Vector3 linkPosOther = link->positionOther(n->id);
                Vector3 linkPos = link->position(n->id);

                delta += constraint.delta;
                M++;
            }

            delta /= M;

            other->moveBy( -delta );

            activeGraph->vs.addVector( other->position(Vec4d(0.5)), delta );
        }

		//double hdistAfter = HausdorffDistance( other->controlPoints(), tother->controlPoints() );

		//if(hdistAfter > hdistBefore)
		{
			//other->setControlPoints( cptsBefore );
		}

        // Modify actual geometry
        double t = otherTask->localT( globalTime * s->totalExecutionTime() );
        otherTask->geometryMorph( qRanged(0.0, t, 1.0) );
    }
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
        
		// The translation needed
        Vec3d delta = link->positionOther(n->id) - link->position(n->id);
        if(delta.norm() < 1e-7) delta = Vector3(0);

        constraints[ otherTask ].push_back( LinkConstraint(delta, link, task, otherTask) );
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
		Vector3 newPos = linkPos + constraint.delta;

		// translate by moving handle
		other->deformTo( handle, newPos, true );

		activeGraph->vs.addVector( linkPos, constraint.delta );
	}

	if( N > 1 )
	{
		Vector3 delta(0);
		int M = 0;

		foreach( LinkConstraint constraint, consts )
		{
			Task * task = constraint.task;
			Structure::Link * link = constraint.link;
			Structure::Node * n = task->node();

			// Check for changing end edges
			if( !link->hasNode(n->id) || !link->hasNode(other->id) )	return;

			delta += constraint.delta;
			M++;
		}

		delta /= M;

		other->moveBy( -delta );

		activeGraph->vs.addVector( other->position(Vec4d(0.5)), delta );
	}

	// Modify actual geometry
	double t = otherTask->localT( globalTime * s->totalExecutionTime() );
	otherTask->geometryMorph( qRanged(0.0, t, 1.0) );
}


//Task* Relink::taskToBeRelinked()
//{
//	Task* nextTask = NULL;
//	double bestCertainty = -1;
//
//	foreach (Task* task, constraints.keys())
//	{
//		int nE = activeGraph->getEdges(task->nodeID).size();
//		int nC = constraints[task].size();
//		double certainty =  nC / (double) nE;
//
//		// return if 100% sure
//		if (certainty == 1.0)
//		{
//			return task;
//		}
//
//		// otherwise pick up the one with best certainty
//		if (certainty > bestCertainty)
//		{
//			bestCertainty = certainty;
//			nextTask = task;
//		}
//	}
//
//	return nextTask;
//}