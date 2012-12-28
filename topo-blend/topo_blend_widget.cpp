#include "topo-blend.h"
#include "topo_blend_widget.h"
#include "ui_topo_blend_widget.h"
#include "ui_animationWidget.h"

#include "QuickMesh.h"
#include "QuickViewer.h"
#include <QFileDialog>

QuickViewer * viewer = NULL;
QString cur_filename;

topo_blend_widget::topo_blend_widget(topoblend * topo_blend, QWidget *parent) : QWidget(parent), ui(new Ui::topo_blend_widget)
{
    ui->setupUi(this);

    this->tb = topo_blend;

	// Generate models
    topo_blend->connect(ui->genChairsButton, SIGNAL(clicked()), SLOT(generateChairModels()));
	topo_blend->connect(ui->genSimpleButton, SIGNAL(clicked()), SLOT(generateTwoSimpleModels()));

	// Save / load graphs
    topo_blend->connect(ui->loadButton, SIGNAL(clicked()), SLOT(loadModel()));
	topo_blend->connect(ui->saveButton, SIGNAL(clicked()), SLOT(saveModel()));
	topo_blend->connect(ui->linksButton, SIGNAL(clicked()), SLOT(modifyModel()));
	topo_blend->connect(ui->clearButton, SIGNAL(clicked()), SLOT(clearGraphs()));

	// Animation widget
	topo_blend->connect(ui->button4, SIGNAL(clicked()), SLOT(currentExperiment()));
	this->connect(ui->animationButton, SIGNAL(clicked()), SLOT(renderViewer()));

	// Main blending process
	topo_blend->connect(ui->blendButton, SIGNAL(clicked()), SLOT(doBlend()));

	// Correspondence
	topo_blend->connect(ui->sourceID, SIGNAL(valueChanged(int)), SLOT(visualizePart2PartDistance(int)));
	topo_blend->connect(ui->one2oneButton, SIGNAL(clicked()), SLOT(findOne2OneCorrespondences()));
	topo_blend->connect(ui->one2manyButton, SIGNAL(clicked()), SLOT(findOne2ManyCorrespondences()));
	topo_blend->connect(ui->p2pButton, SIGNAL(clicked()), SLOT(testPoint2PointCorrespondences()));
	topo_blend->connect(ui->runButton, SIGNAL(clicked()), SLOT(computeCorrespondences()));
	// End of Correspondence
}

topo_blend_widget::~topo_blend_widget()
{
    delete ui;
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

		viewer->m->load(filename);
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
    viewer->m->load(cur_filename);
	viewer->setFocus();
}

