#pragma once
#include "Task.h"

class TaskSheet : public Task
{
public:
    TaskSheet( Structure::Graph * activeGraph, Structure::Graph * targetGraph, TaskType taskType, int ID ) :
        Task(activeGraph, targetGraph, taskType, ID){}

    void prepareSheet();
    void executeSheet(double t);

    /// Prepare Task
    void prepareGrowShrinkSheet();
    void prepareMorphSheet();

    /// Prepare sub-routines
    void prepareSheetOneEdge( Structure::Link * l );
    void prepareSheetTwoEdges( Structure::Link * linkA, Structure::Link * linkB );

    /// Execute
    void executeCrossingSheet( double t );
    void executeMorphSheet( double t );

    // Quick access
    Structure::Sheet * targetSheet();

    // Special encoding for sheet growing
    SheetEncoding encodeSheetAsCurve( Structure::Sheet * sheet, Vector3 start, Vector3 end );
    Array1D_Vector3 decodeSheetFromCurve( double t, SheetEncoding cpCoords, Vector3 start, Vector3 end  );
};
