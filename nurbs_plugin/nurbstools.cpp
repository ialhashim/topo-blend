#include "nurbstools.h"
#include "ui_nurbstools.h"
#include "nurbs_plugin.h"

NURBSTools::NURBSTools(nurbs_plugin * usePlugin, QWidget *parent) : QWidget(parent), ui(new Ui::NURBSTools)
{
    ui->setupUi(this);
    this->plugin = usePlugin;

    // Connect
    plugin->connect(ui->fitCurveButton, SIGNAL(clicked()), SLOT(doFitCurve()));
    plugin->connect(ui->fitSurfaceButton, SIGNAL(clicked()), SLOT(doFitSurface()));

	plugin->connect(ui->clearButton, SIGNAL(clicked()), SLOT(clearAll()));
	plugin->connect(ui->saveButton, SIGNAL(clicked()), SLOT(saveAll()));

	plugin->connect(ui->skeletonButton, SIGNAL(clicked()), SLOT(skeletonizeMesh()));
	plugin->connect(ui->skeletonButtonStep, SIGNAL(clicked()), SLOT(stepSkeletonizeMesh()));

	plugin->connect(ui->curveFitButton, SIGNAL(clicked()), SLOT(convertToCurve()));
	plugin->connect(ui->sheetFitButton, SIGNAL(clicked()), SLOT(convertToSheet()));

	plugin->connect(ui->flipUButton, SIGNAL(clicked()), SLOT(flipU()));
	plugin->connect(ui->flipVButton, SIGNAL(clicked()), SLOT(flipV()));
	
	plugin->connect(ui->experimentButton, SIGNAL(clicked()), SLOT(experiment()));
}

NURBSTools::~NURBSTools()
{
    delete ui;
}

int NURBSTools::uCount()
{
	return ui->uCount->value();
}

int NURBSTools::vCount()
{
	return ui->vCount->value();
}

double NURBSTools::resolution()
{
	return ui->resolution->value();
}

int NURBSTools::contractIterations()
{
	return ui->contractIterations->value();
}

bool NURBSTools::isRemesh()
{
	return ui->remeshOption->isChecked();
}

bool NURBSTools::isVoxelize()
{
	return ui->voxelOption->isChecked();
}

double NURBSTools::remeshParamter()
{
	return ui->remeshParamter->value();
}

double NURBSTools::voxelParamter()
{
	return ui->voxelParamter->value();
}
