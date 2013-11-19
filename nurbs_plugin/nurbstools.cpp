#include "nurbstools.h"
#include "ui_nurbstools.h"
#include "nurbs_plugin.h"

#include "StructureGraph.h"
#include "StructureCurve.h"
#include "StructureSheet.h"
#include "GraphCorresponder.h"

#include "GraphModifyWidget.h"

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

	this->connect(ui->curveFitButton, SIGNAL(clicked()), SLOT(convertToCurve()));
	this->connect(ui->sheetFitButton, SIGNAL(clicked()), SLOT(convertToSheet()));

	this->connect(ui->flipUButton, SIGNAL(clicked()), SLOT(flipU()));
	this->connect(ui->flipVButton, SIGNAL(clicked()), SLOT(flipV()));

	ui->partsList->connect(ui->clearSelectedButton, SIGNAL(clicked()), SLOT(clearSelection()));
	this->connect(ui->partsList, SIGNAL(itemSelectionChanged()), SLOT(selectionChanged()));
	this->connect(ui->linksButton, SIGNAL(clicked()), SLOT(modifyGraph()));
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

void NURBSTools::flipU()
{
	QStringList selected = selectedGroups();

	foreach(QString gid, selectedGroups()){
		Structure::Node * n = plugin->graph->getNode(gid);
		if(!n) continue;
		if(n->type() == Structure::CURVE){
			Array1D_Vector3 cpts = n->controlPoints();
			std::reverse(cpts.begin(), cpts.end());
			n->setControlPoints(cpts);
		}
		else{
			Structure::Sheet * sheet = ((Structure::Sheet*)n);
			Array2D_Vector3 cpts = sheet->surface.mCtrlPoint, newPts = cpts;
			int nU = cpts.size(); int nV = cpts.front().size();
			for(int u = 0; u < nU; u++)
				for(int v = 0; v < nV; v++)
					newPts[u][v] = cpts[(nU - 1) - u][v];
			sheet->surface.mCtrlPoint = newPts;
			sheet->surface.quads.clear();
		}
	}

	ui->partsList->clearSelection();
	plugin->updateDrawArea();
}

void NURBSTools::flipV()
{
	QStringList selected = selectedGroups();

	foreach(QString gid, selectedGroups()){
		Structure::Node * n = plugin->graph->getNode(gid);
		if(!n) continue;
		if(n->type() == Structure::CURVE){
			Array1D_Vector3 cpts = n->controlPoints();
			std::reverse(cpts.begin(), cpts.end());
			n->setControlPoints(cpts);
		}
		else{
			Structure::Sheet * sheet = ((Structure::Sheet*)n);
			Array2D_Vector3 cpts = sheet->surface.mCtrlPoint, newPts = cpts;
			int nU = cpts.size(); int nV = cpts.front().size();
			for(int u = 0; u < nU; u++)
				for(int v = 0; v < nV; v++)
					newPts[u][v] = cpts[u][(nV - 1) - v];
			sheet->surface.mCtrlPoint = newPts;
			sheet->surface.quads.clear();
		}
	}

	ui->partsList->clearSelection();
	plugin->updateDrawArea();
}

void NURBSTools::fillList()
{
	ui->partsList->clear();

	foreach(QString groupID, plugin->groupFaces.keys()){
		QListWidgetItem * item = new QListWidgetItem( groupID );
		
		if(plugin->graph->getNode(groupID)) item->setTextColor(Qt::gray);

		ui->partsList->addItem(item);
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

	plugin->updateDrawArea();
}

void NURBSTools::modifyGraph()
{
	GraphModifyWidget * widget = new GraphModifyWidget(plugin->graph);
	plugin->connect(widget, SIGNAL(updateView()), SLOT(updateDrawArea()));
	widget->show();
}

void NURBSTools::convertToCurve()
{
	QStringList selected = selectedGroups();
	ui->partsList->clearSelection();

	if( selected.isEmpty() ){
		plugin->convertToCurve();
		plugin->updateDrawArea();
		fillList();
		return;
	}

	foreach(QString gid, selected){
		plugin->selectGroup(gid);
		plugin->convertToCurve();
	}

	if(selected.size() < 2) return;

	/// Post process:
	QColor groupColor = qRandomColor2();
	Structure::Node * firstElement = plugin->graph->getNode(selected.front());

	foreach(QString gid, selected)
	{
		// Change color
		Structure::Node * n = plugin->graph->getNode(gid);
		n->vis_property["color"].setValue( groupColor );

		// Align to first in group
		GraphCorresponder::correspondTwoCurves((Structure::Curve*)firstElement, (Structure::Curve*)n, plugin->graph);
	}

	plugin->graph->addGroup(selected.toVector());

	plugin->updateDrawArea();
	fillList();
}

void NURBSTools::convertToSheet()
{
	QStringList selected = selectedGroups();
	ui->partsList->clearSelection();

	if( selected.isEmpty() ){
		plugin->convertToSheet();
		plugin->updateDrawArea();
		fillList();
		return;
	}

	foreach(QString gid, selected){
		plugin->selectGroup(gid);
		plugin->convertToSheet();
	}

	if(selected.size() < 2) return;

	/// Post process:
	QColor groupColor = qRandomColor2();
	Structure::Node * firstElement = plugin->graph->getNode(selected.front());

	foreach(QString gid, selected)
	{
		// Change color
		Structure::Node * n = plugin->graph->getNode(gid);
		n->vis_property["color"].setValue( groupColor );

		// Align to first in group
		GraphCorresponder::correspondTwoSheets((Structure::Sheet*)firstElement, (Structure::Sheet*)n, plugin->graph);
	}

	plugin->graph->addGroup(selected.toVector());

	plugin->updateDrawArea();
	fillList();
}
