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
		Structure::Node * tother = targetGraph->getNode( other->property["correspond"].toString() );

		double hdistBefore = HausdorffDistance( other->controlPoints(), tother->controlPoints() );
		Array1D_Vector3 cptsBefore = other->controlPoints();

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

            bool isRigid = otherTask->isDone;

            if(other->type() == Structure::CURVE){
                if( abs(handle[0] - 0.5) < 0.1 )
                    isRigid = true;
            }

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

		double hdistAfter = HausdorffDistance( other->controlPoints(), tother->controlPoints() );

		if(hdistAfter > hdistBefore)
		{
			other->setControlPoints( cptsBefore );
		}

        // Modify actual geometry
        double t = otherTask->localT( globalTime * s->totalExecutionTime() );
        otherTask->geometryMorph( qRanged(0.0, t, 1.0) );
    }
}
