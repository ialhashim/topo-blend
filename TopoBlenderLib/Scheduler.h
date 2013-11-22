#pragma once

#include "StructureGraph.h"
#include <QGraphicsScene>
#include <QDockWidget>
#include "TimelineSlider.h"

class Task;
class SchedulerWidget;

typedef QMap< QString, QPair<int,int> > ScheduleType;

class Scheduler : public QGraphicsScene
{
    Q_OBJECT

public:
	Scheduler();
	~Scheduler();

	Scheduler(const Scheduler& other);
	Scheduler * clone();

	// Properties
	int rulerHeight;
	bool isForceStop;
	PropertyMap property;
	QVector<Task*> tasks;

	SchedulerWidget * widget;
	TimelineSlider * slider;
	QDockWidget * dock;

	// Execution parameters
	double timeStep;
	double globalStart;
	double globalEnd;
	double overTime;

	// Output
	QVector<Structure::Graph*> allGraphs;

	// Input
	void setInputGraphs(Structure::Graph * source, Structure::Graph * target);
	QMap<QString, QString> superNodeCorr; // correspondence used to generate tasks
	Structure::Graph * activeGraph;
	Structure::Graph * targetGraph;

	Structure::Graph * originalActiveGraph;
	Structure::Graph * originalTargetGraph;

	bool isApplyChangesUI;

public:
	void prepareSynthesis();
	void generateTasks();
	void schedule();
	void order();
    void executeAll();
	void finalize();
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

	void drawDebug();

protected:
	void drawBackground ( QPainter * painter, const QRectF & rect );
	void drawForeground ( QPainter * painter, const QRectF & rect );

	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event );
	void mousePressEvent( QGraphicsSceneMouseEvent * event );
	void mouseMoveEvent( QGraphicsSceneMouseEvent * event );

public slots:
	void timeChanged(int newTime);
	void stopExecution();
    void doBlend();

	void doRenderAll() { emit( renderAll() ); } 
	void doRenderCurrent() { emit( renderCurrent() ); }
	void doDraftRender() { emit(draftRender()); }
		
	void setGDResolution(double r);
	void setTimeStep(double dt);

	void startAllSameTime();
	void startDiffTime();

	void loadSchedule(QString filename);
	void saveSchedule(QString filename);
	void setSchedule( ScheduleType fromSchedule );
	ScheduleType getSchedule();
	static ScheduleType reversedSchedule(const ScheduleType & fromSchedule);

	void defaultSchedule();
	void shuffleSchedule();
	QVector<ScheduleType> manyRandomSchedules(int N);
	QVector<ScheduleType> allSchedules();
	QVector<Structure::Graph*> interestingInBetweens(int N);

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
