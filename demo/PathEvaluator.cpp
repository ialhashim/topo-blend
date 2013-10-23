#include <QDebug>
#include "PathEvaluator.h"
#include "SynthesisManager.h"

PathEvaluator::PathEvaluator( Blender * blender, QObject *parent ) : QObject(parent), b(blender)
{
	b->connect(this, SIGNAL(evaluationDone()), SLOT(generatePaths()));
}

void PathEvaluator::evaluatePaths()
{
	QElapsedTimer timer; timer.start();

	int timeLimit = 10000; // ms
	int numSamplesPerPath = 100;

	// Find time it takes for a single path
	int timePerPath = 0;
	{
		Scheduler s( *b->m_scheduler );
		s.setSchedule( b->allSchedules.front() );
		s.timeStep = 1.0 / numSamplesPerPath;
		s.executeAll();
	}
	timePerPath = timer.elapsed();

	int numPaths = timeLimit / timePerPath;

	for(int i = 0; i < numPaths; i++)
	{
		// Compute path

		// Evaluate path

		// Put in priority queue
	}

	emit( evaluationDone() );
}
