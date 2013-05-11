#include "topo-blend.h"
#include "topo_blend_widget.h"
#include "ui_topo_blend_widget.h"
#include "ui_animationWidget.h"
#include "landmarks_dialog.h"
#include "StarlabDrawArea.h"

#include "QuickMesh.h"
#include "QuickViewer.h"
#include "QuickGroup.h"
#include <QFileDialog>
#include <QDir>
#include <QListWidgetItem>

#include "topo-blend.h"
#include "GraphCorresponder.h"
#include "TopoBlender.h"
#include "Scheduler.h"
#include "SchedulerWidget.h"

QuickViewer * viewer = NULL;
QString cur_filename;

topo_blend_widget::topo_blend_widget(topoblend * topo_blend, QWidget *parent) : QWidget(parent), ui(new Ui::topo_blend_widget)
{
    ui->setupUi(this);
	ui->groupBox1->setVisible(false);

    this->tb = topo_blend;
	
	// Generate models
    //topo_blend->connect(ui->genChairsButton, SIGNAL(clicked()), SLOT(generateChairModels()));
	//topo_blend->connect(ui->genSimpleButton, SIGNAL(clicked()), SLOT(generateTwoSimpleModels()));

	// Save / load graphs
    topo_blend->connect(ui->loadButton, SIGNAL(clicked()), SLOT(loadModel()));
    topo_blend->connect(ui->saveButton, SIGNAL(clicked()), SLOT(saveModel()));
    topo_blend->connect(ui->clearButton, SIGNAL(clicked()), SLOT(clearGraphs()));
    topo_blend->connect(ui->linksButton, SIGNAL(clicked()), SLOT(modifyModel()));
    topo_blend->connect(ui->alignButton, SIGNAL(clicked()), SLOT(quickAlign()));
	topo_blend->connect(ui->normalizeButton, SIGNAL(clicked()), SLOT(normalizeAllGraphs()));

	// Save / load entire jobs
	this->connect(ui->loadJobButton, SIGNAL(clicked()), SLOT(loadJob()));
	this->connect(ui->saveJobButton, SIGNAL(clicked()), SLOT(saveJob()));

	// Animation widget
	//topo_blend->connect(ui->button4, SIGNAL(clicked()), SLOT(currentExperiment()));
	//this->connect(ui->animationButton, SIGNAL(clicked()), SLOT(renderViewer()));

	// Main blending process
    this->connect(ui->blendButton, SIGNAL(clicked()), SLOT(doBlend()), Qt::UniqueConnection);

	// Correspondence
	this->connect(ui->computeCorrButton, SIGNAL(clicked()), SLOT(loadCorrespondenceModel()));

	// Grouping
	this->connect(ui->groupButton, SIGNAL(clicked()), SLOT(showGroupingDialog()));

	// Synthesis
	topo_blend->connect(ui->genSynthButton, SIGNAL(clicked()), SLOT(generateSynthesisData()), Qt::UniqueConnection);
	topo_blend->connect(ui->saveSynthButton, SIGNAL(clicked()), SLOT(saveSynthesisData()));
	topo_blend->connect(ui->loadSynthButton, SIGNAL(clicked()), SLOT(loadSynthesisData()));
	topo_blend->connect(ui->outputCloudButton, SIGNAL(clicked()), SLOT(outputPointCloud()));
	topo_blend->connect(ui->reconstructButton, SIGNAL(clicked()), SLOT(reconstructXYZ()));
	topo_blend->connect(ui->combineMeshesButton, SIGNAL(clicked()), SLOT(combineMeshesToOne()));

	// Model manipulation
	this->connect(ui->normalizeModel, SIGNAL(clicked()), SLOT(normalizeModel()));
	this->connect(ui->bottomCenterModel, SIGNAL(clicked()), SLOT(bottomCenterModel()));
	this->connect(ui->moveModel, SIGNAL(clicked()), SLOT(moveModel()));
	this->connect(ui->rotateModel, SIGNAL(clicked()), SLOT(rotateModel()));
	this->connect(ui->scaleModel, SIGNAL(clicked()), SLOT(scaleModel()));
	this->connect(ui->exportAsOBJ, SIGNAL(clicked()), SLOT(exportAsOBJ()));

	// Manipulate parts
	this->connect(ui->reverseCurveButton, SIGNAL(clicked()), SLOT(reverseCurve()));

	// Populate list
	this->updatePartsList();
	this->connect(ui->refreshViewButton, SIGNAL(clicked()), SLOT(updatePartsList()));

	// Visualization & default values
	this->connect(ui->vizButtonGroup, SIGNAL(buttonClicked(QAbstractButton*)),SLOT(vizButtonClicked(QAbstractButton*)));
	tb->viz_params["showNodes"] = true;
	tb->viz_params["showMeshes"] = true;
	tb->viz_params["splatSize"] = 0.008;

	this->connect(ui->splatSize, SIGNAL(valueChanged(double)), SLOT(splatSizeChanged(double)));
}

topo_blend_widget::~topo_blend_widget()
{
    delete ui;
}

void topo_blend_widget::doBlend()
{
    tb->doBlend();
}

void topo_blend_widget::loadJob()
{
	QFile job_file( QFileDialog::getOpenFileName(0, tr("Load Job"), "", tr("Job Files (*.job)")) );
	if (!job_file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
	QFileInfo jobFileInfo(job_file.fileName());
	QTextStream in(&job_file);

	// Job file content
	QString sgFileName, tgFileName, correspondenceFileName, scheduleFileName;
	int samplesCount;
	double gdResolution, timeStep;
	int reconLevel, renderCount;

	QString curPath = jobFileInfo.absolutePath() + "/";

	sgFileName = in.readLine();
	tgFileName = in.readLine();
	correspondenceFileName = in.readLine();
	scheduleFileName = in.readLine();

	in >> samplesCount;
	in >> gdResolution >> timeStep;
	in >> reconLevel >> renderCount;

	// Load graphs
	tb->graphs.push_back( new Structure::Graph ( curPath + sgFileName ) );
	tb->graphs.push_back( new Structure::Graph ( curPath + tgFileName ) );
	tb->setSceneBounds();
	tb->updateDrawArea();

	// Load correspondence
	tb->corresponder();
	tb->corresponder()->loadCorrespondences( curPath + correspondenceFileName );

	// Create TopoBlender
	tb->corresponder()->isReady = true;  // temporally block computing correspondences
	tb->doBlend();
	tb->corresponder()->isReady = false;

	// Load schedule
	tb->scheduler->loadSchedule( curPath + scheduleFileName );

	// Set parameters
	ui->synthesisSamplesCount->setValue( samplesCount );
	tb->scheduler->widget->setParams( gdResolution, timeStep, reconLevel, renderCount );

	// Sample meshes
	tb->generateSynthesisData();
}

void topo_blend_widget::saveJob()
{
	if(!tb->scheduler) return;

	QString sGraphName = tb->corresponder()->sgName();
	QString tGraphName = tb->corresponder()->tgName();

	QString graph_names = ( sGraphName + "_" + tGraphName ) + ".job";

	QString job_filename = QFileDialog::getSaveFileName(0, tr("Save Job"), graph_names, tr("Job Files (*.job)"));

	QFile job_file( job_filename );
	if (!job_file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QFileInfo jobFileInfo(job_file.fileName());
	QTextStream out(&job_file);

	// Create folders
	QDir jobDir( jobFileInfo.absolutePath() );

	QString sDir = jobDir.path() + "/Source";
	QString tDir = jobDir.path() + "/Target";
	jobDir.mkdir( sDir ); 
	jobDir.mkdir( tDir );

	// Save source and target graphs
	QString sRelative = "Source/" + sGraphName + ".xml";
	QString sgFileName = jobDir.path() + sRelative;
	tb->blender->sg->saveToFile( sgFileName );

	QString tRelative = "Target/" + tGraphName + ".xml";
	QString tgFileName = jobDir.path() + tRelative;
	tb->blender->tg->saveToFile( tgFileName );

	// Save correspondence file
	QString correspondRelative = "correspondence.txt";
	QString correspondenceFileName = jobDir.path() + "/" + correspondRelative;
	tb->corresponder()->saveCorrespondences( correspondenceFileName );

	// Save the scheduler
	QString scheduleRelative = "schedule.txt";
	QString scheduleFileName = jobDir.path() + "/" + scheduleRelative;
	tb->scheduler->saveSchedule( scheduleFileName );

	// Save paths & parameters
	out << sRelative << "\n";
	out << tRelative << "\n";
	out << correspondRelative << "\n";
	out << scheduleRelative << "\n";

	// Synthesis
	out << ui->synthesisSamplesCount->value() << "\t";

	// Discretization 
	out << tb->scheduler->widget->gdResolution() << "\t" << tb->scheduler->widget->timeStep() << "\n";

	// Reconstruction
	out << tb->scheduler->widget->reconLevel() << "\t" << tb->scheduler->widget->renderCount() << "\n";

	job_file.close();
}

void topo_blend_widget::showGroupingDialog()
{
	QuickGroup groupingDialog(tb->graphs.front());
	tb->connect(&groupingDialog, SIGNAL(updateView()), SLOT(updateDrawArea()));
	groupingDialog.exec();
}

void topo_blend_widget::renderViewer()
{
	QDialog * d = new QDialog;
	d->show();

    Ui::AnimationForm aniForm;
    aniForm.setupUi(d);

    viewer = new QuickViewer();
    aniForm.mainLayout->addWidget(viewer);
    viewer->makeCurrent();

    this->connect(aniForm.button, SIGNAL(clicked()), SLOT(renderAnimation()));
    this->connect(aniForm.loadButton, SIGNAL(clicked()), SLOT(loadAnimationModel()));
}

void topo_blend_widget::renderAnimation()
{
	QStringList filters, files;
	filters << "*.obj" << "*.off";
	files = QDir(".").entryList(filters);

	int nframes = 0;

	foreach(QString filename, files)
	{
		QString prefix = QFileInfo(cur_filename).baseName();

		if(!filename.startsWith(prefix.mid(0,3))) continue;

		viewer->m->load(filename, false, false);
		viewer->updateGL();
		viewer->saveSnapshot(filename + ".png");

		nframes++;
	}

	// Generate GIF using ImageMagick
	system(qPrintable( QString("convert	-delay %1	-loop 0		seq*.png	animation.gif").arg( nframes / 20 ) ));

	viewer->setFocus();
}

void topo_blend_widget::loadAnimationModel()
{
    viewer->m = new QuickMesh;
    cur_filename = QFileDialog::getOpenFileName(this, tr("Open Mesh"), "", tr("Mesh Files (*.obj *.off)"));
    viewer->m->load(cur_filename, false, false);

	qglviewer::Vec a = qglviewer::Vec(viewer->m->bbmin.x(), viewer->m->bbmin.y(),viewer->m->bbmin.z());
	qglviewer::Vec b = qglviewer::Vec(viewer->m->bbmax.x(), viewer->m->bbmax.y(),viewer->m->bbmax.z());
	viewer->setSceneBoundingBox(a,b);
	viewer->showEntireScene();

	viewer->setFocus();
}

void topo_blend_widget::loadCorrespondenceModel()
{
	if (tb->corresponder())
	{
		LandmarksDialog dialog(tb);
		dialog.exec();
	}
}

void topo_blend_widget::vizButtonClicked(QAbstractButton* b)
{
	Q_UNUSED(b)

	tb->viz_params["showNodes"] = ui->showNodes->isChecked();
	tb->viz_params["showEdges"] = ui->showEdges->isChecked();
	tb->viz_params["showMeshes"] = ui->showMeshes->isChecked();
	tb->viz_params["showTasks"] = ui->showTasks->isChecked();
	tb->viz_params["showCtrlPts"] = ui->showCtrlPts->isChecked();
	tb->viz_params["showCurveFrames"] = ui->showCurveFrames->isChecked();
	tb->viz_params["isSplatsHQ"] = ui->isSplatsHQ->isChecked();
	tb->viz_params["splatSize"] = ui->splatSize->value();

	tb->updateDrawArea();
}

void topo_blend_widget::splatSizeChanged(double newSize)
{
	tb->viz_params["splatSize"] = newSize;
	tb->updateDrawArea();
}

void topo_blend_widget::toggleCheckOption( QString optionName )
{
	foreach (QAbstractButton *pButton, ui->vizButtonGroup->buttons()){
		if(pButton->objectName() == optionName){
			QCheckBox * checkBox = (QCheckBox *)pButton;
			checkBox->setChecked( !checkBox->isChecked() );
			vizButtonClicked(pButton);
			return;
		}
	}
}

void topo_blend_widget::setCheckOption( QString optionName )
{
	foreach (QAbstractButton *pButton, ui->vizButtonGroup->buttons()){
		if(pButton->objectName() == optionName){
			((QCheckBox *)pButton)->setChecked(true);
			vizButtonClicked(pButton);
			return;
		}
	}
}

int topo_blend_widget::synthesisSamplesCount()
{
	return ui->synthesisSamplesCount->value();
}

void topo_blend_widget::normalizeModel()
{
	if(!tb->graphs.size()) return;

	tb->graphs.back()->normalize();

	tb->setSceneBounds();
	tb->updateDrawArea();
}

void topo_blend_widget::bottomCenterModel()
{
	if(!tb->graphs.size()) return;

	tb->graphs.back()->moveBottomCenterToOrigin();
	tb->setSceneBounds();
	tb->updateDrawArea();
}

void topo_blend_widget::moveModel()
{
	if(!tb->graphs.size()) return;

	double s = 0.1;

	QMatrix4x4 mat;
	mat.translate( ui->movX->value() * s, ui->movY->value() * s, ui->movZ->value() * s );
	tb->graphs.back()->transform( mat );
	tb->setSceneBounds();
	tb->updateDrawArea();
}

void topo_blend_widget::rotateModel()
{
	if(!tb->graphs.size()) return;

	QMatrix4x4 mat;
	mat.rotate( ui->rotX->value(), QVector3D(1,0,0) );
	mat.rotate( ui->rotY->value(), QVector3D(0,1,0) );
	mat.rotate( ui->rotZ->value(), QVector3D(0,0,1) );
	tb->graphs.back()->transform( mat );
	tb->setSceneBounds();
	tb->updateDrawArea();
}

void topo_blend_widget::scaleModel()
{
	if(!tb->graphs.size()) return;

	QMatrix4x4 mat;
	if( ui->isUniformScale->isChecked() )
	{
		double s = qMax(ui->scaleX->value(),qMax(ui->scaleY->value(),ui->scaleZ->value()));
		mat.scale( s );
	}
	else
	{
		mat.scale( ui->scaleX->value(), ui->scaleY->value(), ui->scaleZ->value() );
	}
	tb->graphs.back()->transform( mat );
	tb->setSceneBounds();
	tb->updateDrawArea();
}

void topo_blend_widget::exportAsOBJ()
{
	if(!tb->graphs.size()) return;

	Structure::Graph * g = tb->graphs.back();

	QString filename = QFileDialog::getSaveFileName(0, tr("Export OBJ"), 
		g->property["name"].toString().replace(".xml",".obj"), tr("OBJ Files (*.obj)"));
	
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
	QFileInfo fileInfo(file.fileName());
	QTextStream out(&file);
	out << "# Exported graph model [" << fileInfo.baseName() << "] by TopoBlender\n\n";

	int v_offset = 0;

	foreach(Structure::Node * n, g->nodes)
	{
		if(!n->property.contains("mesh")) continue;

		out << "# Starting mesh " << n->id << "\n";
		
		SurfaceMesh::Model * m = n->property["mesh"].value<SurfaceMesh::Model*>();

		// Write out vertices
		Vector3VertexProperty points = m->vertex_property<Vector3>(VPOINT);
		foreach( Vertex v, m->vertices() )
			out << "v " << points[v][0] << " " << points[v][1] << " " << points[v][2] << "\n";

		// Write out triangles
		out << "g " << n->id << "\n";
		foreach( Face f, m->faces() ){
			out << "f ";
			Surface_mesh::Vertex_around_face_circulator fvit = m->vertices(f), fvend = fvit;
			do{	out << (((Surface_mesh::Vertex)fvit).idx() + 1 + v_offset) << " ";} while (++fvit != fvend);
			out << "\n";
		}

		v_offset += m->n_vertices();

		out << "# End of mesh " << n->id << "\n\n";
	}

	file.close();
}

void topo_blend_widget::reverseCurve()
{
	if(!tb->graphs.size()) return;
	Structure::Graph * g = tb->graphs.back(); 

	foreach(QListWidgetItem * item, ui->partsList->selectedItems())
	{
		Structure::Node * n = g->getNode(item->text());
		if(n->type() != Structure::CURVE) continue;
		Structure::Curve * curve = (Structure::Curve*) n;

		// Flip the selected curve
		std::vector<Vector3> ctrlPoint = curve->controlPoints();
		std::vector<Scalar> ctrlWeight = curve->controlWeights();
		std::reverse(ctrlPoint.begin(), ctrlPoint.end());
		std::reverse(ctrlWeight.begin(), ctrlWeight.end());

		NURBS::NURBSCurved newCurve(ctrlPoint, ctrlWeight);
		curve->curve = newCurve;

		// Update the coordinates of its links
		foreach( Structure::Link * l, g->getEdges(curve->id) )
		{
			l->setCoord(curve->id, inverseCoords( l->getCoord(curve->id) ));
		}
	}

	tb->updateDrawArea();
}

void topo_blend_widget::updatePartsList()
{
	if(!tb->graphs.size()) return;
	Structure::Graph * g = tb->graphs.back();

	// Populate list
	foreach(Structure::Node * n, g->nodes) 
		ui->partsList->addItem(new QListWidgetItem(n->id));
}
