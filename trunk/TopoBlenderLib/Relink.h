#pragma once
#include "StructureGraph.h"
#include "Task.h"
#include "QQueue"

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

	bool checkRelinkability;
	QQueue<Task*> propagationQueue;

	void execute();
	void fixTask( Task* task );
	void propagateFrom( Task* task );

	// Helpers
	Vector3 getToDelta( Structure::Link * link, QString otherID );
	bool isRelinkable( Task* task );
};
