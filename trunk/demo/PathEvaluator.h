#pragma once
#include <QObject>

#include "Blender.h"

class PathEvaluator : public QObject
{
    Q_OBJECT
public:
    explicit PathEvaluator(Blender * blender, QObject *parent = 0);

public slots:
	// Old experiments
	void evaluatePaths();
	void clusterPaths();

	// Current experiments
	void test_filtering();
	void test_topoDistinct();

private:
	Blender * b;

signals:
    void evaluationDone();
};
