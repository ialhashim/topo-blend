#include <omp.h>

#include <QDebug>
#include "PathEvaluator.h"
#include "SynthesisManager.h"
#include "RelationManager.h"

#include "BlendPathRenderer.h"

PathEvaluator::PathEvaluator( Blender * blender, QObject *parent ) : QObject(parent), b(blender)
{
	
}

void PathEvaluator::evaluatePaths()
{
	int numSamplesPerPath = 100;
	int timeLimit = 5000; // ms

	/// Get number of paths to evaluate:
	int numPaths = 0;
	{
		// Find time it takes for a single path
		QElapsedTimer timer; timer.start();
		{
			Scheduler s( *b->m_scheduler );
			s.setSchedule( b->m_scheduler->getSchedule() ); // default
			s.timeStep = 1.0 / numSamplesPerPath;
			s.executeAll();
		}

		numPaths = qMax(1.0, double(timeLimit) / timer.elapsed() * omp_get_num_threads());
	}

	// Force number of paths
	numPaths = 30;

	b->emitMessage("Number of paths to evaluate: " + QString::number(numPaths));

	/// Setup schedules:
	QVector<Scheduler*> randomSchedules;
	
	foreach(ScheduleType curSched, b->m_scheduler->manyRandomSchedules( numPaths )){
		Scheduler * s = new Scheduler( *b->m_scheduler );
		s->setSchedule( curSched );
		s->timeStep = 1.0 / numSamplesPerPath;

		randomSchedules.push_back(s);
	}

	/// Path evaluation:
	std::vector<double> scheduleScore(numPaths, 0);
	{
		QVector<Structure::Graph*> inputGraphs;
		inputGraphs << b->s->inputGraphs[0]->g << b->s->inputGraphs[1]->g;

		// Do this once for input graphs
		RelationManager r_manager(b->m_gcorr, b->m_scheduler.data(), inputGraphs);
		r_manager.parseModelConstraintPair( false );
		r_manager.parseModelConstraintGroup( false );

		for(int i = 0; i < numPaths; i++)
		{
			Scheduler * s = randomSchedules[i];

			// Compute path
			s->executeAll();

			// Evaluate path
			double score = r_manager.traceModelConstraints( s->allGraphs );

			// Put in priority queue
			scheduleScore[i] = score;
		}
	}

	// Sort by score
	std::vector<unsigned int> indices;
	paired_sort<double>(indices, scheduleScore);

	QVector<Scheduler*> sortedSchedules;
	foreach(unsigned int idx, indices) sortedSchedules.push_back( randomSchedules[idx] );

	///////////////////////////////////////////////////////////

	/// Export results
	QString sessionName = QString("session_%1").arg(QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
	QString path = "results/" + sessionName;
	QDir d("");	d.mkpath( path ); d.mkpath( path + "/images" );

	// HTML header
	QStringList html;
	html << "<!DOCTYPE html>";
	html << "<html>" << "<head>" << "<title> Results </title> <link rel='stylesheet' href='../style.css'>" << "</head>" << "<body>";
	html << "<h1>" << sessionName << "</h1>";
	html << "<h2 id ='input'>" << "Parents" << "</h2>";

	// Inputs
	QColor inputColor(255,255,255);
	b->renderer->quickRender(sortedSchedules.front()->allGraphs.front(), inputColor).save(path + "/images/source.png");
	b->renderer->quickRender(sortedSchedules.front()->allGraphs.back(), inputColor).save(path + "/images/target.png");
	html << QString("<div> <img src='images/source.png'/> <img src='images/target.png'/> </div>");

	// Outputs
	html << "<div id='results'>" << "<table>";

	int numInBetweens = 8;

	for(int i = 0; i < (int)indices.size() * 0.5; i++)
	{
		int j = (indices.size() - 1) - i;

		html << "<tr>";

		// High
		{
			Scheduler * s = sortedSchedules[j];
			double score = scheduleScore[j];

			html << "<td>" << "<div class='score-high'>" << QString::number(score) << "</div>";
			html << "<div class='path-img'>";
			
			QVector<Structure::Graph*> inBetweens = s->interestingInBetweens( numInBetweens );
			for(int k = 0; k < numInBetweens; k++)
			{
				QString imageName = QString("%1_%2.png").arg(j).arg(k);
				b->renderer->quickRender(inBetweens[k], inputColor).save(path + "/images/" + imageName);
				html << QString(" <img src='images/%1'/> ").arg(imageName);
			}

			html << "</div>" << "</td>";
		}

		// Low
		{
			Scheduler * s = sortedSchedules[i];
			double score = scheduleScore[i];

			html << "<td>" << "<div class='score-low'>" << QString::number(score) << "</div>";
			html << "<div class='path-img'>";

			QVector<Structure::Graph*> inBetweens = s->interestingInBetweens( numInBetweens );
			for(int k = 0; k < numInBetweens; k++)
			{
				QString imageName = QString("%1_%2.png").arg(i).arg(k);
				b->renderer->quickRender(inBetweens[k], inputColor).save(path + "/images/" + imageName);
				html << QString(" <img src='images/%1'/> ").arg(imageName);
			}

			html << "</div>" << "</td>";
		}

		html << "</tr>";
	}

	// Footer
	html << "</table>" << "</div>" << "</body>" << "</html>";

	QString filename( path + "/index.html"  );
	QFile file(filename); file.open(QIODevice::WriteOnly | QIODevice::Text);
	QTextStream out(&file);
	out << html.join("\n");
	file.close();

	emit( evaluationDone() );
}
