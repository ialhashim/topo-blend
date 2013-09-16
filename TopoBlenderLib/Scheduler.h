#pragma once

#include "StructureGraph.h"
#include <QGraphicsScene>
#include <QDockWidget>
#include "TimelineSlider.h"

class Task;
class SchedulerWidget;

class Scheduler : public QGraphicsScene
{
    Q_OBJECT

public:
	Scheduler();
	~Scheduler();
	PropertyMap property;

	// Input
	QVector<Task*> tasks;

	Structure::Graph * activeGraph;
	Structure::Graph * targetGraph;
	QMap<QString, QString> superNodeCorr; // correspondence used to generate tasks

	SchedulerWidget * widget;
	QDockWidget * dock;

    void prepareSynthesis();

	// Output
	QVector<Structure::Graph*> allGraphs;
	
	void generateTasks();
	void schedule();
	void order();
	void executeAll();
	void reset();

	void blendDeltas( double globalTime, double timeStep );
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

	// Execution parameters
	double timeStep;
	double globalStart;
	double globalEnd;

	void drawDebug();

protected:
	void drawBackground ( QPainter * painter, const QRectF & rect );
	void drawForeground ( QPainter * painter, const QRectF & rect );

	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event );
	void mousePressEvent( QGraphicsSceneMouseEvent * event );
	void mouseMoveEvent( QGraphicsSceneMouseEvent * event );

public slots:
	void timeChanged(int newTime);
	void doBlend();
	void stopExecution();

	void startAllSameTime();
	void startDiffTime();
	void defaultSchedule();
	void shuffleSchedule();

	void doRenderAll() { emit( renderAll() ); } 
	void doRenderCurrent() { emit( renderCurrent() ); }
	void doDraftRender() { emit(draftRender()); }
		
	void setGDResolution(double r);
	void setTimeStep(double dt);

	void loadSchedule(QString filename);
	void saveSchedule(QString filename);
	QMap< QString, QPair<int,int> > getSchedule();
	void setSchedule( QMap< QString, QPair<int,int> > fromSchedule );
	
	void emitUpdateExternalViewer();
	void emitProgressStarted();
	void emitProgressChanged(int);
	void emitProgressedDone();

signals:
	void activeGraphChanged( Structure::Graph* );
	void startBlend();

	void progressStarted();
	void progressChanged(int);
	void progressDone();

	void renderAll();
	void renderCurrent();
	void draftRender();

	void updateExternalViewer();
	void hasReset();
};
