#pragma once
#include <QObject>

#include "Blender.h"

struct PathScore{
	double score;
	QStringList images;
	PathScore() { score = 0.0; }
	bool operator<(const PathScore& other) const { return score < other.score; }
};

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
