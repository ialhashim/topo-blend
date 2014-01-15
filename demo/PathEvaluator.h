#pragma once
#include <QObject>

#include "Blender.h"

class PathEvaluator : public QObject
{
    Q_OBJECT
public:
    explicit PathEvaluator(Blender * blender, QObject *parent = 0);

public slots:
	// Current experiments
	void test_filtering();
	void test_topoDistinct();

	QVector<ScheduleType> filteredSchedules( QVector<ScheduleType> randomSchedules );

	void evaluateFilter( QVector<Structure::Graph*> allGraphs );

private:
	Blender * b;

signals:
    void evaluationDone();
	void progressChanged(double);
};
