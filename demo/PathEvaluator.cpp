#include <omp.h>
#include <QDebug>

#include "PathEvaluator.h"
#include "SynthesisManager.h"

#include "BlendPathRenderer.h"

#include "json.h"

PathEvaluator::PathEvaluator( Blender * blender, QObject *parent ) : QObject(parent), b(blender)
{
	
}

void PathEvaluator::evaluatePaths()
{
	/// Export results:
	QString sessionName = QString("session_%1").arg(QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
	QString path = "results/" + sessionName;
	QDir d("");	d.mkpath( path ); d.mkpath( path + "/images" );

	int timeLimit = 5000; // ms
	int numInBetweens = 8;
	int numSamplesPerPath = 100;

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
	numPaths = 100;

	/// Setup schedules:
	QVector<PathScore> ps(numPaths);

	// Do this once for input graphs
	QVector<Structure::Graph*> inputGraphs;
	inputGraphs << b->s->inputGraphs[0]->g << b->s->inputGraphs[1]->g;

	//RelationManager r_manager(b->m_gcorr, b->m_scheduler.data(), inputGraphs);
	//r_manager.parseModelConstraintPair( false );
	//r_manager.parseModelConstraintGroup( false );

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
		//ps[i].scores = r_manager.traceModelConstraints( s.allGraphs );
		
		double maxScorePath = ps[i].maxScore();

		//#pragma omp critical
		{
			QImage img(QSize(600,130), QImage::Format_ARGB32);
			img.fill(qRgba(0,0,0,0));

			QPainter painter(&img);
			painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);
			
			// Graph
			QPainterPath poly;
			int padding = 20;
			poly.moveTo(img.width(), padding);
			poly.lineTo(img.width(), img.height());
			poly.lineTo(0, img.height());
			poly.lineTo(0, padding);

			// Render path
			QVector<Structure::Graph*> inBetweens = s.interestingInBetweens( numInBetweens );
			for(int k = 0; k < numInBetweens; k++)
			{
				QImage inBetween = b->renderer->quickRender(inBetweens[k], Qt::white);

				// Rendered shape
				int newWidth = inBetween.width() * 0.5;
				int startX = k * newWidth;
				painter.drawImage(startX, 0, inBetween);

				// Score graph
				double val = inBetweens[k]->property["score"].toDouble() / maxScorePath;
				int x = startX + newWidth;
				int y = img.height() - (img.height() * val);
				poly.lineTo(x, y + padding);
			}

			poly.lineTo(img.width(), padding);
			painter.setPen(QPen(Qt::black, 3));
			painter.setBrush(QColor(50,255,50,30));
			painter.drawPath(poly);

			img.save( path + QString("/images/%1.png").arg(i) );
		}
	}

	// Sort based on score
	qSort(ps);

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
			html << "<td>" << QString("<div class='score-%1'>").arg(scoreType[h]) << QString::number(ps[ idx[h] ].score()) << "</div>";
			html << "<div class='path-img'>";
			html << QString(" <img src='images/%1.png'/> ").arg( idx[h] );
			html << "</div>" << "</td>";
		}

		html << "</tr>";
	}

	// Footer
	html << "</table>" << "</div>";
	html << "<h2>Time taken: " << QString::number((timeElapsed / 1000) / 60) << " minutes</h2>";
		
	html << "</body>" << "</html>";

	QString filename( path + "/index.html"  );
	QFile file(filename); file.open(QIODevice::WriteOnly | QIODevice::Text);
	QTextStream out(&file);
	out << html.join("\n");
	file.close();

	emit( evaluationDone() );
}

void PathEvaluator::clusterPaths()
{
	int numPaths = 100;
	int numInBetweens = 20;
	int numSamplesPerPath = 100;

	// Exporting
	QString sessionName = QString("cluster_%1").arg(QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
	QString path = "results/" + sessionName;
	QDir d("");	d.mkpath( path ); d.mkpath( path + "/images" );

	QtJson::JsonObject json;
	QtJson::JsonArray allData;
	
	QVector< QVector<double> > rawData;
	int idx = 0;
	
	QVector<ScheduleType> allPaths = b->m_scheduler->manyRandomSchedules( numPaths );

	for(int i = 0; i < allPaths.size(); i++)
	{
		// Setup schedule
		Scheduler s( *b->m_scheduler );
		s.setSchedule( allPaths[i] );
		s.timeStep = 1.0 / numSamplesPerPath;

		// Execute blend
		s.executeAll();

		QtJson::JsonArray b_path;

		QVector<Structure::Graph*> inBetweens = s.interestingInBetweens( numInBetweens );
		
		for(int k = 0; k < numInBetweens; k++)
		{
			QtJson::JsonObject tvals, ibetween;
			QVector<double> rawDataInbetweens;

			foreach(Structure::Node * n, inBetweens[k]->nodes)
			{
				double t = n->property["t"].toDouble();
				tvals[n->id] = t;

				rawDataInbetweens.push_back(t);
			}
		
			ibetween["t"] = tvals;
			ibetween["image"] = QString("images/%1_%2.png").arg(i).arg(k);
			ibetween["id"] = idx++;

			//b->renderer->quickRender(inBetweens[k], Qt::white).save(path + "/" + ibetween["image"].toString());

			b_path.append(ibetween);

			rawData.push_back(rawDataInbetweens);
		}

		allData.append(b_path);
	}

	json["data"] = allData;

	/// Export results:
	{
		QString filename( path + "/data.json"  );
		QFile file(filename); file.open(QIODevice::WriteOnly | QIODevice::Text);
		QTextStream out(&file);
		out << QtJson::serialize(allData);
		file.close();
	}

	// Draw data
	{
		QString filename( path + "/rawdata.csv"  );
		QFile file(filename); file.open(QIODevice::WriteOnly | QIODevice::Text);
		QTextStream out(&file);
		foreach(QVector<double> shapeVector, rawData){
			QStringList tvector;
			foreach(double t, shapeVector) tvector << QString::number(t);
			out << tvector.join(",") << "\n";
		}
		file.close();
	}
}
