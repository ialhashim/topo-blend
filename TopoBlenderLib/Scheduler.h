#pragma once

#include "StructureGraph.h"
#include <QGraphicsScene>
#include "TimelineSlider.h"
class Task;

class Scheduler : public QGraphicsScene
{
    Q_OBJECT

public:
    Scheduler();

	// Input
	QVector<Task*> tasks;

	Structure::Graph * activeGraph;
	Structure::Graph * sourceGraph;
	Structure::Graph * targetGraph;

    void prepareSynthesis();

	// Output
	QVector<Structure::Graph*> allGraphs;
	
	void schedule();
	void order();
	void executeAll();

	int totalExecutionTime();

	// Dependency
	QVector<QString> activeTasks(double globalTime);

	// Helper functions
	QVector<Task*> tasksSortedByStart();
	Task * getTaskFromNodeID( QString nodeID );

	QList<Task*> sortTasksByPriority( QList<Task*> curTasks );
	QList<Task*> sortTasksAsLayers( QList<Task*> currentTasks, int startTime = 0 );

	void groupStart( Structure::Graph * g, QList<Task*> curTasks, int curStart, int & futureStart );

	bool isPartOfGrowingBranch( Task* t );
	QVector<Task*> getEntireBranch( Task * t );

	// Task operations
	void addMorphTask( QString nodeID );

	// Time helpers
	void splitTasksStartTime( int startTime, QList<Task*> & before, QList<Task*> & after );
	void slideTasksTime( QList<Task*> list_tasks, int delta );
	int startOf( QList<Task*> list_tasks );
	int endOf( QList<Task*> list_tasks );

	// Properties
	int rulerHeight;
	bool isForceStop;

	TimelineSlider * slider;

	void drawDebug();

protected:
	void drawBackground ( QPainter * painter, const QRectF & rect );
	void drawForeground ( QPainter * painter, const QRectF & rect );

public slots:
	void timeChanged(int newTime);
	void doBlend();
	void stopExecution();

	void startAllSameTime();

	void doRenderAll() { emit( renderAll() ); } 
	void doRenderCurrent() { emit( renderCurrent() ); }
	void doDraftRender() { emit(draftRender()); }

signals:
	void activeGraphChanged( Structure::Graph* );
	void startBlend();

	void progressStarted();
	void progressChanged(int);
	void progressDone();

	void renderAll();
	void renderCurrent();
	void draftRender();
};
