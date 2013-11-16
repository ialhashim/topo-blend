#include "nurbstools.h"
#include "ui_nurbstools.h"
#include "nurbs_plugin.h"

NURBSTools::NURBSTools(nurbs_plugin * usePlugin, QWidget *parent) : QWidget(parent), ui(new Ui::NURBSTools)
{
    ui->setupUi(this);
    this->plugin = usePlugin;

    // Connect
    //plugin->connect(ui->fitCurveButton, SIGNAL(clicked()), SLOT(doFitCurve()));
    //plugin->connect(ui->fitSurfaceButton, SIGNAL(clicked()), SLOT(doFitSurface()));

	plugin->connect(ui->clearButton, SIGNAL(clicked()), SLOT(clearAll()));
	plugin->connect(ui->saveButton, SIGNAL(clicked()), SLOT(saveAll()));

	//plugin->connect(ui->skeletonButton, SIGNAL(clicked()), SLOT(skeletonizeMesh()));
	plugin->connect(ui->skeletonButtonStep, SIGNAL(clicked()), SLOT(stepSkeletonizeMesh()));

	plugin->connect(ui->curveFitButton, SIGNAL(clicked()), SLOT(convertToCurve()));
	plugin->connect(ui->sheetFitButton, SIGNAL(clicked()), SLOT(convertToSheet()));

	plugin->connect(ui->flipUButton, SIGNAL(clicked()), SLOT(flipU()));
	plugin->connect(ui->flipVButton, SIGNAL(clicked()), SLOT(flipV()));

	ui->partsList->connect(ui->clearSelectedButton, SIGNAL(clicked()), SLOT(clearSelection()));
	this->connect(ui->partsList, SIGNAL(itemSelectionChanged()), SLOT(selectionChanged()));
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

bool NURBSTools::isUseMedial()
{
	return ui->medialOption->isChecked();
}

bool NURBSTools::isProtectSmallFeatures()
{
	return ui->protectSmallOption->isChecked();
}

double NURBSTools::remeshParamter()
{
	return ui->remeshParamter->value();
}

double NURBSTools::voxelParamter()
{
	return ui->voxelParamter->value();
}

void NURBSTools::fillList()
{
	ui->partsList->clear();

	foreach(QString groupID, plugin->groupFaces.keys()){
		ui->partsList->addItem(groupID);
	}
}

QStringList NURBSTools::selectedGroups()
{
	QStringList selected;
	foreach( QListWidgetItem *item, ui->partsList->selectedItems()){
		selected << item->text();
	}
	return selected;
}

void NURBSTools::selectionChanged()
{
	// Draw selection
	{
		plugin->ps.clear();

		foreach(QString gid, selectedGroups()){
			QVector<int> part = plugin->groupFaces[gid];

			foreach(int fidx, part)
			{
				// Collect points
				QVector<QVector3> pnts; 
				Surface_mesh::Vertex_around_face_circulator vit = plugin->entireMesh->vertices(Face(fidx)),vend=vit;
				do{ pnts.push_back(plugin->entirePoints[vit]); } while(++vit != vend);

				// Add triangle
				plugin->ps.addPoly(pnts, QColor(255,0,0,100));
			}
		}
	}
}
