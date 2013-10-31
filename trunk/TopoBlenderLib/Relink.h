#pragma once
#include "StructureGraph.h"
#include "Task.h"
#include <QQueue>

struct LinkConstraint{
    Structure::Link *link;
    Task *task, *otherTask;
    LinkConstraint(Structure::Link * l=NULL, Task* t=NULL, Task* otherT=NULL)
    { link = l; task = t; otherTask = otherT; }
};
typedef QMap< Task*, QVector<LinkConstraint> > TasksConstraints;

class Scheduler;

struct Relink
{
    Relink( Scheduler * scheduler );

    Scheduler * s;
    Structure::Graph *activeGraph, *targetGraph;
	
    TasksConstraints constraints;

	// Tracking
	typedef QPair<QString,int> PropagationEdge;
	typedef QVector< PropagationEdge > PropagationEdges;
	QMap< QString, PropagationEdges > propagationGraph;
	int propagationIndex;

	void execute();
	void fixTask( Task* task );

	// Helpers
	Vector3 getToDelta( Structure::Link * link, QString otherID );
	bool doesPropagate( Task* task );
	bool isInActiveGroup( Task* task );
};
