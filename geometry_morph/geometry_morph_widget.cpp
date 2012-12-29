#include "geometry_morph.h"
#include "geometry_morph_widget.h"
#include "ui_geometry_morph_widget.h"
#include "ui_geometry_animationWidget.h"

#include "QuickMesh2.h"
#include "QuickViewer2.h"
#include <QFileDialog>

QuickViewer * viewer = NULL;
QString cur_filename;

geometry_morph_widget::geometry_morph_widget(geometry_morph * geo_morph, QWidget *parent) : QWidget(parent), ui(new Ui::geometry_morph_widget)
{
	ui->setupUi(this);

	this->gm = geo_morph;

	geo_morph->connect(ui->button1, SIGNAL(clicked()), SLOT(loadSourceModel()));
	geo_morph->connect(ui->button2, SIGNAL(clicked()), SLOT(loadTargetModel()));

	geo_morph->connect(ui->button3, SIGNAL(clicked()), SLOT(loadSourceCurve()));
	geo_morph->connect(ui->button4, SIGNAL(clicked()), SLOT(loadTargetCurve()));
	geo_morph->connect(ui->button5, SIGNAL(clicked()), SLOT(doMorph()));

	this->connect(ui->button6, SIGNAL(clicked()), SLOT(renderViewer()));

	geo_morph->connect(ui->button7, SIGNAL(clicked()), SLOT(loadSourceGraph()));
	geo_morph->connect(ui->button8, SIGNAL(clicked()), SLOT(loadTargetGraph()));

}

geometry_morph_widget::~geometry_morph_widget()
{
	delete ui;
}

void geometry_morph_widget::renderViewer()
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

void geometry_morph_widget::renderAnimation()
{
	QStringList filters, files;
	filters << "*.obj" << "*.off";
	files = QDir(".").entryList(filters);

	int nframes = 0;
	bool SET_VIEWPORT=false;

	foreach(QString filename, files)
	{
		QString prefix = QFileInfo(cur_filename).baseName();

		if(!filename.startsWith(prefix.mid(0,3))) continue;

		viewer->m->load(filename,SET_VIEWPORT);
		viewer->updateGL();
		viewer->saveSnapshot(filename + ".png");

		SET_VIEWPORT=true;
		nframes++;
}
}

void geometry_morph_widget::loadAnimationModel()
{
	viewer->m = new QuickMesh;
	cur_filename = QFileDialog::getOpenFileName(this, tr("Open Mesh"), "", tr("Mesh Files (*.obj *.off)"));
	viewer->m->load(cur_filename,false);
	viewer->setFocus();
}
