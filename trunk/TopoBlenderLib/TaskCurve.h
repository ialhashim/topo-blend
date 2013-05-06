#pragma once
#include "Task.h"

class TaskCurve : public Task
{
public:
    TaskCurve( Structure::Graph * activeGraph, Structure::Graph * targetGraph, TaskType taskType, int ID ) :
        Task(activeGraph, targetGraph, taskType, ID){}

    void prepareCurve();
    void executeCurve(double t);

    /// Prepare Task
    void prepareGrowCurve();
    void prepareShrinkCurve();
    void prepareMorphCurve();

    /// Prepare sub-routines
    void prepareShrinkCurveOneEdge( Structure::Link* l );
	void prepareCrossingMorphCurve();

    /// Execute
    void executeCrossingCurve( double t );
    void executeMorphCurve( double t );
    void foldCurve( double t );

    // Quick access
    Structure::Curve * targetCurve();
};
