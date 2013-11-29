#pragma once
#include <QObject>

#include "Blender.h"

struct PathScore{
	QVector< QVector<double> > scores;
	double score() const { return minScore(0); }
	double maxScore(int i) const { return *std::max_element(scores[i].begin(), scores[i].end()); }
	double minScore(int i) const { return *std::min_element(scores[i].begin(), scores[i].end()); }
	bool operator<(const PathScore& other) const { return score() < other.score(); }
};

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
