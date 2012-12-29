#include "geometry_morph.h"
#include "geometry_morph_widget.h"
#include "ui_geometry_morph_widget.h"
#include <QFileDialog>

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

	geo_morph->connect(ui->button7, SIGNAL(clicked()), SLOT(loadSourceGraph()));
	geo_morph->connect(ui->button8, SIGNAL(clicked()), SLOT(loadTargetGraph()));

}

geometry_morph_widget::~geometry_morph_widget()
{
	delete ui;
}

void geometry_morph_widget::renderViewer()
{

}

void geometry_morph_widget::renderAnimation()
{

}

void geometry_morph_widget::loadAnimationModel()
{

}
