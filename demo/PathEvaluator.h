#pragma once
#include <QObject>

#include "Blender.h"

struct PathScore{
	QVector<double> scores;
	double score() const { return minScore(); }
	double maxScore() const { return *std::max_element(scores.begin(), scores.end()); }
	double minScore() const { return *std::min_element(scores.begin(), scores.end()); }
	bool operator<(const PathScore& other) const { return score() < other.score(); }
};

class PathEvaluator : public QObject
{
    Q_OBJECT
public:
    explicit PathEvaluator(Blender * blender, QObject *parent = 0);

public slots:
	void evaluatePaths();
	void clusterPaths();

private:
	Blender * b;

signals:
    void evaluationDone();
};
