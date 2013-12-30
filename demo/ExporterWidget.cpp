#include <QVariant>

#include "ExporterWidget.h"
#include "Blender.h"
#include "SynthesisManager.h"
#include "BlendRenderItem.h"
#include "StructureGraph.h"
#include "ShapeRenderer.h"

#include "HttpUploader.h"

#include "ui_ExporterWidget.h"

Q_DECLARE_METATYPE( ScheduleType )

QString html( QString insideHTML, QString tag, QString stylesheet = ""){
	return QString("<%1 style='%3'>%2</%1>").arg(tag).arg(insideHTML).arg(stylesheet);
}
QString htmlRow( QStringList columns ){
	return QString("<tr>%1</tr>").arg("<td>" + columns.join("</td><td>") + "</td>" );
}
QString htmlTable( QVector<QStringList> rows, QString stylesheet ){
	QString table;
	foreach(QStringList columns, rows){
		table += htmlRow( columns );
	}
	return html(table, "table", stylesheet);
}

ExporterWidget::ExporterWidget(Session *session, QWidget *parent) : QWidget(parent), session(session), ui(new Ui::ExporterWidget)
{
    ui->setupUi(this);
	this->setVisible(false);

#ifdef QT_DEBUG
	ui->isFullRecon->setChecked(false);
#endif

	// Connections
	this->connect(ui->addButton, SIGNAL(clicked()), SLOT(addInBetween()));
	this->connect(ui->removeButton, SIGNAL(clicked()), SLOT(removeInBetween()));
	this->connect(ui->exportButton, SIGNAL(clicked()), SLOT(exportSet()));

	this->connect(session->s, SIGNAL(keyUpEvent(QKeyEvent*)), SLOT(keyUp(QKeyEvent*)));

	setWindowFlags(windowFlags() | Qt::WindowStaysOnTopHint);
}

ExporterWidget::~ExporterWidget()
{
    delete ui;
}

void ExporterWidget::keyUp(QKeyEvent* event)
{
	if(event->key() == Qt::Key_E)
	{
		if( isVisible() )
		{
			session->s->setProperty("grabMatcher", false);
			this->setVisible(false);
		}
		else
		{
			session->s->setProperty("grabMatcher", true);
			this->setVisible(true);
		}
	}
}

void ExporterWidget::addInBetween()
{
	foreach(BlendRenderItem * item, session->b->selectedInBetween())
	{
		Structure::Graph * g = new Structure::Graph(*item->graph());
		QString sname = g->property["sourceName"].toString();
		QString tname = g->property["targetName"].toString();
		QString filename = sname + "." + tname;

		QVariant graph_p;
		graph_p.setValue( g );

		QTableWidgetItem *newItem = new QTableWidgetItem( filename );
		newItem->setData(Qt::UserRole, graph_p);
		newItem->setData(Qt::DecorationRole, item->pixmap.scaled(100, 100, Qt::KeepAspectRatio, Qt::SmoothTransformation));

		ui->resultsTable->insertRow(ui->resultsTable->rowCount());
		ui->resultsTable->setItem(ui->resultsTable->rowCount() - 1, 0, newItem);
	}

	ui->resultsTable->resizeColumnsToContents();
	ui->resultsTable->resizeRowsToContents();

	session->b->clearSelectedInBetween();

	ui->resultsTable->setCurrentCell(ui->resultsTable->rowCount() - 1,0);
}

void ExporterWidget::removeInBetween()
{
	if(ui->resultsTable->rowCount() < 1) return;

	QSet<int> toRemove;

	foreach(QTableWidgetItem * item, ui->resultsTable->selectedItems())
	{
		ui->resultsTable->removeRow( item->row() );
	}
}

void ExporterWidget::exportSet()
{
	if(ui->resultsTable->rowCount() < 1) return;

	SynthesisManager * s_manager = session->b->s_manager.data();
	if(!s_manager) return;

	// Post reconstruction simplification or compression
	QString simplifyCmd, uploadCmd;
	if( ui->isSimplify->isChecked() )
	{
		QSettings settingsFile(QSettings::IniFormat, QSettings::UserScope, "GrUVi", "demo");
		simplifyCmd = settingsFile.value("simplifyCmd", "ctmconv.exe %1.obj %2.ctm --no-normals --no-texcoords --no-colors --method MG2").toString();
		settingsFile.sync();
		simplifyCmd = QInputDialog::getText(this, "Simplification command", "Execute command", QLineEdit::Normal, simplifyCmd);
	}

	if( ui->isUpload->isChecked() )
	{
		QSettings settingsFile(QSettings::IniFormat, QSettings::UserScope, "GrUVi", "demo");
		uploadCmd = settingsFile.value("uploadCmd", "zip %1.zip -r %2").toString();
		settingsFile.sync();
		uploadCmd = QInputDialog::getText(this, "Upload command", "Execute command", QLineEdit::Normal, uploadCmd);
	}

	qApp->setOverrideCursor(Qt::WaitCursor);

	// Logging:
	QMap<QString, QVariant> log;
	QElapsedTimer samplingTimer, reconTimer, allTimer;
	allTimer.start();

	// Reconstruction parameters
	int requestedSamplesCount = ui->samplesCount->value() * 1000;
	int reconLevel = ui->reconLevels->value();

	log["sampling-count"] = requestedSamplesCount;
	log["reconstruction-level"] = reconLevel;

	// Re-sample if needed
	if( s_manager->samplesCount != requestedSamplesCount && ui->isFullRecon->isChecked() )
	{
		logMessage( QString("Sampling (%1 K sample rate)...").arg(ui->samplesCount->value()) );

		samplingTimer.start();
		session->b->property["isOverrideSynth"] = true;

		s_manager->samplesCount = requestedSamplesCount;
		s_manager->genSynData(); // HEAVY

		session->b->property["isOverrideSynth"] = false;
		log["sampling-time"] = (int)samplingTimer.elapsed();

		logMessage( "Done sampling." );
	}

	QString sessionName = QString("session_%1").arg(QDateTime::currentDateTime().toString("dd.MM.yyyy_hh.mm.ss"));
	QStringList filenames;

	QString path = "results/" + sessionName + "/";
	QDir d("");	d.mkpath( path );

	QString sname,tname;

	reconTimer.start();

	for(int i = 0; i < ui->resultsTable->rowCount(); ++i)
	{
		logMessage( QString("Making shape (%1)...").arg(i) );

		QTableWidgetItem *rowItem = ui->resultsTable->item(i, 0);

		QVariant graph_p = rowItem->data(Qt::UserRole);
		Structure::Graph * g = graph_p.value<Structure::Graph*>();
		
		sname = g->property["sourceName"].toString();
		tname = g->property["targetName"].toString();

		QString numString = QString("%1").arg(i, 3, 10, QChar('0'));

		QString filename = sname + "." + tname + "." + numString;
		filenames << (filename);

		d.mkpath( path + filename );

		QDir::setCurrent( d.absolutePath() + "/" + path + filename );

		// Generate the geometry and export the structure graph
		s_manager->renderGraph(*g, filename, false, reconLevel, true, !ui->isSimplify->isChecked());

		// Change camera location if requested
		GLVertex camDelta;
		if( ui->isMoveCamera->isChecked() ) camDelta = GLVertex( ui->moveCamX->value(), ui->moveCamY->value(), ui->moveCamZ->value() );

		// Generate thumbnail
		QString objFile = QDir::currentPath() + "/" + filename + ".obj";
		QString thumbnailFile = QDir::currentPath() + "/" + filename + ".png";
		ShapeRenderer::render( objFile, true, camDelta ).save( thumbnailFile );

		// Output schedule for this in-between
		ScheduleType schedule = g->property["schedule"].value<ScheduleType>();
		Scheduler::saveSchedule(QDir::currentPath() + "/" + filename + ".schedule.txt", schedule);

		// Simplify if requested
		if( ui->isSimplify->isChecked() && !simplifyCmd.isEmpty() )
		{
			QString curCmd = "..\\..\\..\\" + simplifyCmd.arg( filename ).arg( filename );

			qDebug() << curCmd;

			system( qPrintable(curCmd) );

			// delete large OBJ file
			system( "del *.obj" );

			logMessage( QString("Shape (%1) simplified.").arg(i) );
		}

		// Leave folder
		QDir::setCurrent( d.absolutePath() );

		rowItem->setBackgroundColor(QColor(0,100,0));
		ui->resultsTable->setCurrentCell(i,0);

		logMessage( QString("Shape (%1) done.").arg(i) );

		qApp->processEvents();
	}

	ui->resultsTable->clearSelection();
	ui->resultsTable->clearFocus();

	log["sname"] = sname;
	log["tname"] = tname;

	// Source and target
	{
		QString imgPath = d.absolutePath() + "/" + path;
		QString srcFilename = imgPath + QString("%1.png").arg(log["sname"].toString());
		QString tgtFilename = imgPath + QString("%2.png").arg(log["tname"].toString());

		QImage matcherImage = session->s->property("matcherImage").value<QImage>();
		matcherImage.save("matcher.png");

		matcherImage.copy( matcherImage.rect().adjusted(0,0,-matcherImage.width() * 0.5,0) ).save(srcFilename);
		matcherImage.copy( matcherImage.rect().adjusted(matcherImage.width() * 0.5,0,0,0) ).save(tgtFilename);

		// Write '.color' file
		{
			QStringList graphNames;
			graphNames << log["sname"].toString() << log["tname"].toString();

			for(int i = 0; i < 2; i++)
			{
				QString colorFilename = d.absolutePath() + "/" + path + graphNames[i] + ".color";
				QFile file( colorFilename );
				if (file.open(QIODevice::WriteOnly | QIODevice::Text)){
					QTextStream out(&file);
					foreach(Structure::Node * n, session->s->inputGraphs[i]->g->nodes){
						QColor color = n->property["correspondenceColor"].value<QColor>();
						out << QString("%1\t%2\t%3\t%4\n").arg(n->id).arg(color.red()).arg(color.green()).arg(color.blue());
					}
				}
				file.close();

				// Export as segmented OBJ
				session->s->inputGraphs[i]->g->exportAsOBJ( d.absolutePath() + "/" + path + graphNames[i] + ".obj" );
			}
		}

		// Write correspondence file
		{
			QString correspondenceFilename = d.absolutePath() + "/" + path + "correspondence.txt";
			session->m->gcorr->saveCorrespondences( correspondenceFilename );
		}
	}

	log["results"] = filenames;
	log["session"] = sessionName;
	log["reconstruction-time"] = (int)reconTimer.elapsed();

	log["time"] = allTimer.elapsed();
	log["isSimplify"] = ui->isSimplify->isChecked();
	log["isUpload"] = ui->isUpload->isChecked();

	generateLog( log );

	if( ui->isUpload->isChecked() && !uploadCmd.isEmpty() )
	{
		QString curCmd = uploadCmd.arg( sessionName ).arg( path );
		qDebug() << curCmd;
		system( qPrintable(curCmd) );

		QString zipFilename = QString("%1.zip").arg(sessionName);
		logMessage( QString("Upload session (%1) = %2").arg(sessionName).arg( HttpUploader::upload( "http://gruvi.cs.sfu.ca/p/topo/upload.php", zipFilename )) );
	}

	qApp->restoreOverrideCursor();
	QCursor::setPos(QCursor::pos());

	logMessage( "All done." );

	QMessageBox::information(0, "Export", "All jobs done!");
}

void ExporterWidget::generateLog(QMap<QString, QVariant> log)
{
	// Generate '.txt' and '.html' and '.tex'
	QString path = "results/" + log["session"].toString() + "/";
	QDir d(path);

	QString sessionName = log["session"].toString();
	QString filePath = d.absolutePath() + "/";

	int limitPerRow = 3;
	int tableWidth = 1000;

	// Write '.txt' log file
	{
		QString textFilename = filePath + sessionName + ".txt";
		QFile file( textFilename );
		if (file.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QTextStream out(&file);
			foreach(QString key, log.keys())
			{
				QString str_value = log[key].toString();
				QString typeName = log[key].typeName();

				if(typeName == "QStringList")
					str_value = "{" + log[key].toStringList().join(",") + "}";

				out << key << "\t" << str_value << "\n";
			}
		}
		file.close();
	}

	// Write '.html' file
	{
		QString htmlFilename = filePath + sessionName + ".html";
		QFile file( htmlFilename );
		if (file.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QTextStream out(&file);

			QVector<QStringList> rows;

			// Header row
			{
				QStringList firstRow;
				firstRow << QString("<img style='width:" + QString::number(tableWidth / limitPerRow) + "px' src='%1.png' alt='source'/>").arg(log["sname"].toString());
				firstRow << QString("<img style='width:" + QString::number(tableWidth / limitPerRow) + "px' src='%1.png' alt='target'/>").arg(log["tname"].toString());
				rows << firstRow;
			}

			// Results
			{
				QStringList resultsRow;
				QStringList results = log["results"].toStringList();

				foreach(QString r, results){
					resultsRow << QString("<a href='%3.obj'><img style='width:" + QString::number(tableWidth / limitPerRow) + "px' src='%1.png' alt='%2'/></a>").arg(r + "/" + r).arg(r).arg(r + "/" + r);

					// Split at fourth item
					if(resultsRow.size() > limitPerRow){
						rows << resultsRow;
						resultsRow.clear();
					}
				}

				if(resultsRow.size())
					rows << resultsRow;
			}

			out << htmlTable(rows, QString("width:%1px").arg(tableWidth));
		}
		file.close();
	}

	// Write '.tex' file
	{
		QString htmlFilename = filePath + sessionName + ".tex";
		QFile file( htmlFilename );
		if (file.open(QIODevice::WriteOnly | QIODevice::Text))
		{
			QTextStream out(&file);
			out << "\\documentclass{article} \n\\usepackage{graphicx} \n\\usepackage{subfig} \n\\usepackage{grffile}\n \n\\begin{document}\n\n\\begin{figure}[htb]\n";

			double widthPercent = (1.0 / limitPerRow) * 0.95;

			QStringList results = log["results"].toStringList();

			// Source and target
			{
				out << "\n\\subfloat{\\includegraphics[width=" + QString::number(widthPercent) + "\\textwidth]{" + log["sname"].toString() + "}} \\\n";
				out << "\\subfloat{\\includegraphics[width=" + QString::number(widthPercent) + "\\textwidth]{" + log["tname"].toString() + "}} \\\\\n";
			}

			foreach(QString r, results){
				out << "\\subfloat{\\includegraphics[width=" + QString::number(widthPercent) + "\\textwidth]{" + r + "/" + r + "}} \\\n";
			}

			out << "\n\\end{figure}\n\n\\end{document}\n";
		}
		file.close();
	}
}

void ExporterWidget::logMessage( QString message )
{
	ui->log->appendPlainText( message );
}
