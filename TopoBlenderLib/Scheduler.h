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

signals:
	void activeGraphChanged( Structure::Graph* );
	void startBlend();

	void progressStarted();
	void progressChanged(int);
	void progressDone();
};
