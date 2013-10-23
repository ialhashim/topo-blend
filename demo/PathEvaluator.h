#pragma once
#include <QObject>

#include "Blender.h"

class PathEvaluator : public QObject
{
    Q_OBJECT
public:
    explicit PathEvaluator(Blender * blender, QObject *parent = 0);

public slots:
	void evaluatePaths();

private:
	Blender * b;

signals:
    void evaluationDone();
};
