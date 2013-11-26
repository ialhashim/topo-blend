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
	void prepareCrossingSheet();

    /// Execute
	void executeGrowShrinkSheet(double t);
    void executeMorphSheet( double t );
	void executeCrossingSheet( double t );

    // Encoding for sheets as curve
    SheetEncoding encodeSheetAsCurve( Structure::Sheet * sheet, Vector3 start, Vector3 end );
	void encodeSheet( const Vector4d& coordinateA, const Vector4d& coordinateB );

	// Quick access
	Structure::Sheet * targetSheet();
};
