#include <omp.h>
#include <QDebug>

#include "PathEvaluator.h"
#include "SynthesisManager.h"
#include "GraphDistance.h"
#include "BlendPathRenderer.h"
#include "json.h"

#include "ScorerManager.h"

#include "GraphDissimilarity.h"
#include "ExportDynamicGraph.h"

Q_DECLARE_METATYPE( Vector3 )

PathEvaluator::PathEvaluator( Blender * blender, QObject *parent ) : QObject(parent), b(blender)
{
	
}

void PathEvaluator::test_filtering()
{
	/// Export results:
	QString sessionName = QString("session_%1").arg(QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
	QString path = "results/" + sessionName;
	QDir d("");	d.mkpath( path ); d.mkpath( path + "/images" );

	int timeLimit = 5000; // ms
	int numInBetweens = 8;
	int numSamplesPerPath = numInBetweens * 3;

	/// Get number of paths to evaluate:
	int numPaths = 0;
	{
		Scheduler defaultSchedule( *b->m_scheduler );

		// Find time it takes for a single path
		QElapsedTimer timer; timer.start();
		{
			defaultSchedule.setSchedule( b->m_scheduler->getSchedule() ); // default
			defaultSchedule.timeStep = 1.0 / numSamplesPerPath;
			defaultSchedule.executeAll();
		}

		numPaths = qMax(1.0, double(timeLimit) / timer.elapsed() * omp_get_num_threads());

		// Export source and target images
		QColor inputColor(255,255,255);
		b->renderer->quickRender(defaultSchedule.allGraphs.front(), inputColor).save(path + "/images/source.png");
		b->renderer->quickRender(defaultSchedule.allGraphs.back(), inputColor).save(path + "/images/target.png");
	}

	// Force number of paths
	numPaths = 30;

	// Do this once for input graphs
	QVector<Structure::Graph*> inputGraphs;
	inputGraphs << b->s->inputGraphs[0]->g << b->s->inputGraphs[1]->g;

	ScorerManager r_manager(b->m_gcorr, b->m_scheduler.data(), inputGraphs);
	r_manager.parseConstraintPair();
	r_manager.parseConstraintGroup();
	r_manager.parseGlobalReflectionSymm();

	QVector<ScorerManager::PathScore> ps( numPaths );
	MatrixXd allRanges( numPaths, 3 * 4 );

	QVector<ScheduleType> allPaths = b->m_scheduler->manyRandomSchedules( numPaths );

	QElapsedTimer evalTimer; evalTimer.start();

	//#pragma omp parallel for
	for(int i = 0; i < allPaths.size(); i++)
	{
		// Setup schedule
		Scheduler s( *b->m_scheduler );
		s.setSchedule( allPaths[i] );
		s.timeStep = 1.0 / numSamplesPerPath;

		// Execute blend
		s.executeAll();

		// Compute its score
		ps[i] = r_manager.pathScore( &s );

		QVector<QColor> colors;
		colors.push_back(QColor(255,0,0));
		colors.push_back(QColor(0,255,0));
		colors.push_back(QColor(0,0,255));

		// Range of values
		MatrixXd ranges = ps[i].computeRange();

		allRanges.row(i) = VectorXd::Map(ranges.data(), ranges.rows()*ranges.cols());

		//#pragma omp critical
		{
			QImage img(QSize(600,130), QImage::Format_ARGB32);
			img.fill(qRgba(0,0,0,0));

			QPainter painter(&img);
			painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

			// Text option
			QFont font("Monospace",8);
			font.setStyleHint(QFont::TypeWriter);
			painter.setFont(font);

			QVector<Structure::Graph*> inBetweens = s.topoVaryingInBetweens( numInBetweens );

			QVector< QVector<double> > scores(numInBetweens);

			int imgWidth = 0;

			for(int k = 0; k < numInBetweens; k++)
			{
				inBetweens[k]->moveCenterTo( AlphaBlend(inBetweens[k]->property["t"].toDouble(), 
					inBetweens[k]->property["sourceGraphCenter"].value<Vector3>(), 
					inBetweens[k]->property["targetGraphCenter"].value<Vector3>()), true);

				// Draw images
				QImage inBetween = b->renderer->quickRender(inBetweens[k], Qt::white);
				imgWidth = inBetween.width();

				// Rendered shape
				int newWidth = inBetween.width() * 0.5;
				int startX = k * newWidth;
				painter.drawImage(startX, 0, inBetween);

				// Store scores
				int idx = inBetweens[k]->property["graphIndex"].toInt();
				QVector<double> vals;
				vals << ps[i].connectivity[idx] << ps[i].localSymmetry[idx] << ps[i].globalSymmetry[idx];

				scores[k] = vals;
			}
			
			// Draw score lines
			for(int u = 0; u < scores.front().size(); u++)
			{
				// Graph
				QPainterPath poly;
				int padding = 0;

				QColor clr = colors[u];
				painter.setPen(QPen(clr, 1));

				// Render path
				for(int k = 0; k < numInBetweens; k++)
				{
					// Score graph
					//double minVal = ranges(u,0);
					//double range = ranges(u,2);

					//double val = (scores[k][u] - minVal) / range;
					double val = scores[k][u];

					// Graph line
					int newWidth = imgWidth * 0.5;
					int startX = k * newWidth;
					int x = startX + newWidth;
					int y = padding + (img.height() - (img.height() * val));

					if(k == 0)
						poly.moveTo(x, y);
					else
						poly.lineTo(x, y);

					// Dots
					painter.drawEllipse(QPoint(x,y), 2, 2);
				}

				painter.setBrush(Qt::NoBrush);
				painter.drawPath(poly);
			}

			// Draw ranges
			QStringList vals;
			for(int r = 0; r < 3; r++){
				QStringList curVals;
				for(int c = 0; c < 4; c++)
					curVals << QString::number(ranges(r,c),'f',2);
				vals << curVals.join("  ");
			}

			if( false )
			{
				painter.setPen(Qt::white);
				painter.drawText(11, img.height() - 9, vals.join("   #   "));

				painter.setPen(Qt::black);
				painter.drawText(10, img.height() - 10, vals.join("   #   "));
			}

			img.save( path + QString("/images/%1.png").arg(i) );
		}
	}

	// Get global average for the measures
	Vector3d globalAvg (allRanges.col(9).mean(), allRanges.col(10).mean(), allRanges.col(11).mean());

	// Sort based on score
	QMap<int,double> scoreMap;
	for(int i = 0; i < numPaths; i++) scoreMap[i] = ps[i].score();

	QVector<int> sortedIndices;
	typedef QPair<double,int> ValIdx;
	foreach(ValIdx d, sortQMapByValue( scoreMap )){
		sortedIndices.push_back( d.second );
	}

	int timeElapsed = evalTimer.elapsed();
	b->emitMessage( QString("Time (%1 ms), number of paths (%2)").arg(timeElapsed).arg(QString::number(numPaths)) );

	///////////////////////////////////////////////////////////
	// HTML header
	QStringList html;
	html << "<!DOCTYPE html>";
	html << "<html>" << "<head>" << "<title> Results </title> <link rel='stylesheet' href='../style.css'>" << "</head>" << "<body>";
	html << "<h1>" << sessionName << "</h1>";
	html << "<h2 id ='input'>" << "Parents:" << b->m_blender->super_sg->property["sourceName"].toString() << ", "
		<< b->m_blender->super_sg->property["targetName"].toString() << "</h2>";

	// Inputs
	html << QString("<div> <img src='images/source.png'/> <img src='images/target.png'/> </div>");

	// Legend
	html << QString("<div style='color:white; border:1px soild black'> <span style='background:red'>connectivity</span> - \
		<span style='background:green'>local symmetry</span> - <span style='background:blue'>global symmetry</span> </div>");

	// Outputs
	html << "<div id='results'>" << "<table>";

	for(int i = 0; i < numPaths * 0.5; i++)
	{
		int j = (numPaths - 1) - i;
		QStringList scoreType; scoreType << "low" << "high";
		QVector<int> idx; idx << i << j;

		html << "<tr>";

		for(int h = 0; h < 2; h++)
		{
			int index = sortedIndices[idx[h]];

			html << "<td>" << QString("<div class='score-%1'>").arg(scoreType[h]) << QString::number(ps[ index ].score( )) << "</div>";
			html << "<div class='path-img'>";
			html << QString(" <img src='images/%1.png'/> ").arg( index );
			html << "</div>" << "</td>";
		}

		html << "</tr>";
	}

	// Statistics
	html << "<h2> <table>";

	QStringList rowTitles;
	rowTitles << "min-connectivity" << "min-local" << "min-global";
	rowTitles << "max-connectivity" << "max-local" << "max-global";
	rowTitles << "ran-connectivity" << "ran-local" << "ran-global";
	rowTitles << "avg-connectivity" << "avg-local" << "avg-global";

	for(int c = 0; c < allRanges.cols(); c++)
	{
		QStringList stat;
		stat << "min " << QString::number(allRanges.col(c).minCoeff(),'f',2);
		stat << "max " << QString::number(allRanges.col(c).maxCoeff(),'f',2);
		stat << "avg " << QString::number(allRanges.col(c).mean(),'f',2);
		html << "<tr>" << QString("<td>%1</td>").arg( rowTitles[c] ) << "<td>" << stat.join("</td><td>") << "</td></tr>";
	}
		
	html << "</table> </h2>";

	// Footer
	html << "</table>" << "</div>";
	html << "<h2>Time taken: " << QString::number((timeElapsed / 1000) / 60) << " minutes</h2>";

	QString filename( path + "/index.html"  );
	QFile file(filename); file.open(QIODevice::WriteOnly | QIODevice::Text);
	QTextStream out(&file);
	out << html.join("\n");
	file.close();

	emit( evaluationDone() );
}

void PathEvaluator::test_topoDistinct()
{
	QElapsedTimer evalTimer; evalTimer.start();

	/// Export path for results:
	QString sessionName = QString("session_%1").arg(QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
	QString path = "results/" + sessionName;
	QDir d("");	d.mkpath( path ); d.mkpath( path + "/images" );

	/// number of paths to evaluate:
	int numPaths = 15;
	int numSamplesPerPath = 100;
	int numInBetweens = 10;

	/// Export source and target images
	Scheduler defaultSchedule( *b->m_scheduler );
	{
		defaultSchedule.setSchedule( b->m_scheduler->getSchedule() ); // default
		defaultSchedule.timeStep = 1.0 / numSamplesPerPath;
		defaultSchedule.executeAll();
	
		QColor inputColor(255,255,255);
		b->renderer->quickRender(defaultSchedule.allGraphs.front(), inputColor).save(path + "/images/source.png");
		b->renderer->quickRender(defaultSchedule.allGraphs.back(), inputColor).save(path + "/images/target.png");
	}

	/// Generate paths
	QVector<ScheduleType> allPaths = b->m_scheduler->manyRandomSchedules( numPaths );

	//#pragma omp parallel for
	for(int i = 0; i < allPaths.size(); i++)
	{
		// Setup schedule
		Scheduler s( *b->m_scheduler );
		s.setSchedule( allPaths[i] );
		s.timeStep = 1.0 / numSamplesPerPath;

		// Execute blend
		s.executeAll();

		// Compute dissimilarity
		QVector< QVector<double> > diffs;
		QVector<Structure::Graph*> allActualGraphs;
		QVector<double> maxDiffs;
		
		foreach( Structure::Graph* g, s.allGraphs )
			allActualGraphs.push_back( Structure::Graph::actualGraph(g) );

		/// Dissimilarity measure:		
		GraphDissimilarity differ( defaultSchedule.allGraphs.front() );
		differ.addGraph( Structure::Graph::actualGraph(defaultSchedule.allGraphs.front()) );
		differ.addGraph( Structure::Graph::actualGraph(defaultSchedule.allGraphs.back()) );
		differ.addGraphs( allActualGraphs );

		diffs.push_back( differ.computeDissimilar( 0 ) );
		//diffs.push_back( differ.competeDissimilar( 1 ) );

		diffs.back().push_front(0);
		diffs.back().push_back(differ.compute(0,1));

		maxDiffs.push_back( *std::max_element(diffs[0].begin(), diffs[0].end()) );
		//maxDiffs.push_back( *std::max_element(diffs[1].begin(), diffs[1].end()) );

		QVector<QColor> colors;
		colors.push_back(QColor(0,255,0));
		colors.push_back(QColor(0,0,255));
		colors.push_back(QColor(255,0,0));

		// Draw graphs
		{
			int width = (numInBetweens + 1) * (b->renderer->width() * 0.5);
			int height = 130 + 60;
			QImage img( QSize(width, height), QImage::Format_ARGB32 );
			img.fill(qRgba(0,0,0,0));

			QPainter painter(&img);
			painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

			// Text option
			QFont font("Monospace",9);
			font.setStyleHint(QFont::TypeWriter);
			painter.setFont(font);

			QVector<Structure::Graph*> inBetweens = s.topoVaryingInBetweens(numInBetweens);

			// Draw images
			int imgWidth = 0;
			for(int k = 0; k < numInBetweens; k++)
			{
				QImage inBetween = b->renderer->quickRender(inBetweens[k], Qt::white);

				imgWidth = inBetween.width();

				// Rendered shape
				int newWidth = inBetween.width() * 0.5;
				int startX = k * newWidth;
				painter.drawImage(startX, 0, inBetween);
			}

			// Draw score lines
			for(int u = 0; u < diffs.size(); u++)
			{
				// Graph
				QPainterPath poly;
				int padding = 0;

				QColor clr = colors[u];
				clr.setAlphaF(0.9);

				painter.setPen(QPen(clr, 1));
				painter.setBrush(QBrush(clr));

				// Render path
				for(int k = 0; k < numInBetweens; k++)
				{
					int idx = inBetweens[k]->property["graphIndex"].toInt();

					double val = diffs[u][ idx ] / maxDiffs[u];

					// Graph line
					int graphHeight = 100;
					int newWidth = imgWidth * 0.5;
					int startX = k * newWidth;
					int x = startX + newWidth;
					int y = padding + (graphHeight - (graphHeight * val));

					if(k == 0)
						poly.moveTo(x, y);
					else
						poly.lineTo(x, y);

					painter.drawEllipse(QPoint(x,y), 3, 3);

					// Save graph data
					Structure::Graph * g = allActualGraphs[idx];
					painter.drawText(x, height - 50, QString("d %1").arg(QString::number(val,'f',2)));
					painter.drawText(x, height - 40, QString("n %1").arg(g->nodes.size()));
					painter.drawText(x, height - 30, QString("e %1").arg(g->edges.size()));
					painter.drawText(x, height - 20, QString("a %1").arg(g->articulationPoints().size()));
					painter.drawText(x, height - 10, QString("l %1").arg(g->leaves().size()));
				}
				
				painter.setBrush(Qt::NoBrush);
				painter.drawPath(poly);
			}

			img.save( path + QString("/images/%1.png").arg(i) );
		}

		qDeleteAll( allActualGraphs );
	}

	///////////////////////////////////////////////////////////
	// HTML header
	QStringList html;
	html << "<!DOCTYPE html>";
	html << "<html>" << "<head>" << "<title> Results </title> <link rel='stylesheet' href='../style.css'>" << "</head>" << "<body>";
	html << "<h1>" << sessionName << "</h1>";
	html << "<h2 id ='input'>" << "Parents:" << b->m_blender->super_sg->property["sourceName"].toString() << ", "
		<< b->m_blender->super_sg->property["targetName"].toString() << "</h2>";

	// Inputs
	html << QString("<div> <img src='images/source.png'/> <img src='images/target.png'/> </div>");

	// Outputs
	html << "<div id='results'>" << "<table>";

	for(int i = 0; i < allPaths.size(); i++)
	{
		html << "<tr>";
		html << "<td>";
		html << "<div class='path-img'>";
		html << QString(" <img src='images/%1.png'/> ").arg( i );
		html << "</div>";
		html << "</td>";
		html << "</tr>";
	}

	// Footer
	html << "</table>" << "</div>";
	html << "<h2>Time taken: " << QString::number((evalTimer.elapsed() / 1000) / 60) << " minutes</h2>";

	html << "</body>" << "</html>";

	QString filename( path + "/index.html"  );
	QFile file(filename); file.open(QIODevice::WriteOnly | QIODevice::Text);
	QTextStream out(&file);
	out << html.join("\n");
	file.close();

	emit( evaluationDone() );
}

QVector<ScheduleType> PathEvaluator::filteredSchedules( QVector<ScheduleType> randomSchedules )
{
	QVector<ScheduleType> sorted;

	int numSamplesPerPath = 25;
	int numPaths = randomSchedules.size();

	QVector<Structure::Graph*> inputGraphs;
	inputGraphs << b->s->inputGraphs[0]->g << b->s->inputGraphs[1]->g;

	ScorerManager r_manager(b->m_gcorr, b->m_scheduler.data(), inputGraphs);
	r_manager.parseConstraintPair();
	r_manager.parseConstraintGroup();
	r_manager.parseGlobalReflectionSymm();

	QVector<ScorerManager::PathScore> ps( numPaths );
	MatrixXd allRanges( numPaths, 3 * 4 );

	// Optimization
	{
		beginFastNURBS();
		DIST_RESOLUTION = 0.03;
	}

	#pragma omp parallel for
	for(int i = 0; i < numPaths; i++)
	{
		// Setup schedule
		Scheduler s( *b->m_scheduler );
		s.setSchedule( randomSchedules[i] );

		s.timeStep = 1.0 / numSamplesPerPath;

		// Execute blend
		s.executeAll();

		// Compute its score
		ps[i] = r_manager.pathScore( &s );
	}

	// Optimization
	{
		endFastNURBS();
		DIST_RESOLUTION = 0.015;
	}

	// Sort based on score
	QMap<int,double> scoreMap;
	QVector<int> sortedIndices;
	typedef QPair<double,int> ValIdx;

	for(int i = 0; i < numPaths; i++) scoreMap[i] = ps[i].score();

	foreach(ValIdx d, sortQMapByValue( scoreMap ))
		sortedIndices.push_back( d.second );
	
	// Add to sorted list
	for(int i = 0; i < numPaths; i++)
		sorted.push_back( randomSchedules[ sortedIndices[i] ] );

	return sorted;
}
