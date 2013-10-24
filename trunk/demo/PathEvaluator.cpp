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
	/// Export results:
	QString sessionName = QString("session_%1").arg(QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
	QString path = "results/" + sessionName;
	QDir d("");	d.mkpath( path ); d.mkpath( path + "/images" );

	int numSamplesPerPath = 100;
	int timeLimit = 5000; // ms
	int numInBetweens = 8;

	/// Get number of paths to evaluate:
	int numPaths = 0;
	Scheduler defaultSchedule( *b->m_scheduler );
	{
		// Find time it takes for a single path
		QElapsedTimer timer; timer.start();
		{
			defaultSchedule.setSchedule( b->m_scheduler->getSchedule() ); // default
			defaultSchedule.timeStep = 1.0 / numSamplesPerPath;
			defaultSchedule.executeAll();
		}

		numPaths = qMax(1.0, double(timeLimit) / timer.elapsed() * omp_get_num_threads());
	}

	// Force number of paths
	numPaths = 100;

	/// Setup schedules:
	QVector<PathScore> ps(numPaths);

	// Do this once for input graphs
	QVector<Structure::Graph*> inputGraphs;
	inputGraphs << b->s->inputGraphs[0]->g << b->s->inputGraphs[1]->g;

	RelationManager r_manager(b->m_gcorr, b->m_scheduler.data(), inputGraphs);
	r_manager.parseModelConstraintPair( false );
	r_manager.parseModelConstraintGroup( false );

	QVector<ScheduleType> allPaths = b->m_scheduler->manyRandomSchedules( numPaths );

	QElapsedTimer evalTimer; evalTimer.start();

	#pragma omp parallel for
	for(int i = 0; i < allPaths.size(); i++)
	{
		// Setup schedule
		Scheduler s( *b->m_scheduler );
		s.setSchedule( allPaths[i] );
		s.timeStep = 1.0 / numSamplesPerPath;

		// Execute blend
		s.executeAll();
		
		// Compute its score
		ps[i].score = r_manager.traceModelConstraints( s.allGraphs );

		//#pragma omp critical
		{
			// Render path
			QVector<Structure::Graph*> inBetweens = s.interestingInBetweens( numInBetweens );
			for(int k = 0; k < numInBetweens; k++)
			{
				QString imageName = QString("%1_%2.png").arg(i).arg(k);
				QString imagePath = path + "/images/" + imageName;
				//b->renderer->quickRender(inBetweens[k], Qt::white).save(imagePath);
				ps[i].images << imageName;
			}
		}
	}

	// Sort based on score
	qSort(ps);

	b->emitMessage( QString("Time (%1 ms), number of paths (%2)").arg(evalTimer.elapsed()).arg(QString::number(numPaths)) );

	///////////////////////////////////////////////////////////
	// HTML header
	QStringList html;
	html << "<!DOCTYPE html>";
	html << "<html>" << "<head>" << "<title> Results </title> <link rel='stylesheet' href='../style.css'>" << "</head>" << "<body>";
	html << "<h1>" << sessionName << "</h1>";
	html << "<h2 id ='input'>" << "Parents" << "</h2>";

	// Inputs
	QColor inputColor(255,255,255);
	b->renderer->quickRender(defaultSchedule.allGraphs.front(), inputColor).save(path + "/images/source.png");
	b->renderer->quickRender(defaultSchedule.allGraphs.back(), inputColor).save(path + "/images/target.png");
	html << QString("<div> <img src='images/source.png'/> <img src='images/target.png'/> </div>");

	// Outputs
	html << "<div id='results'>" << "<table>";

	for(int i = 0; i < ps.size() * 0.5; i++)
	{
		int j = (ps.size() - 1) - i;
		QStringList scoreType; scoreType << "low" << "high";
		QVector<int> idx; idx << i << j;

		html << "<tr>";

		for(int h = 0; h < 2; h++)
		{
			html << "<td>" << QString("<div class='score-%1'>").arg(scoreType[h]) << QString::number(ps[ idx[h] ].score) << "</div>";
			html << "<div class='path-img'>";

			for(int k = 0; k < numInBetweens; k++)
				html << QString(" <img src='images/%1'/> ").arg(ps[ idx[h] ].images[k]);

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
