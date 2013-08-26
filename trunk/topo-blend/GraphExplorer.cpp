#include <QtConcurrentRun>

#include "GraphExplorer.h"
#include "ui_GraphExplorer.h"

#include "StructureGraph.h"
#include "Scheduler.h"
#include "Task.h"

#include "ExportDynamicGraph.h"

GraphExplorer::GraphExplorer(QWidget *parent): QWidget(parent), ui(new Ui::GraphExplorer)
{
	//setAttribute(Qt::WA_DeleteOnClose);
	ui->setupUi(this);

	g = NULL;
	clear();
}

void GraphExplorer::update(Structure::Graph * graph)
{
	clear();

	g = graph;

	// Set graph name
	QString graphName = g->name().split("/").last();
	ui->graphName->setText( graphName );

	// Graph visualization
	drawGraph();

	// Tree view properties
	QStringList headers;
	headers << "Name" << "Value";
	ui->nodesTree->setHeaderLabels(headers);
	ui->edgesTree->setHeaderLabels(headers);

	// Fill in node and edges info
	fillNodesInfo();
	fillEdgesInfo();

	this->show();
}

void GraphExplorer::drawGraph()
{
	// Create graph visualization
	QString tmpGraphFilename = "tempGraph_"+QString::number(QDateTime::currentMSecsSinceEpoch());
	visualizeStructureGraph(g, tmpGraphFilename, "");
	QImage gimg(tmpGraphFilename+".png");
	QFile::remove(tmpGraphFilename+".png");
	QFile::remove(tmpGraphFilename+".gv");

	// Set graph visualization
	ui->graphImage->setPixmap( QPixmap::fromImage(gimg.scaled(gimg.size() * 0.5, Qt::KeepAspectRatio, Qt::SmoothTransformation)) );
}

void GraphExplorer::fillNodesInfo()
{
	foreach(Structure::Node * n, g->nodes){
		QTreeWidgetItem * nitem = new QTreeWidgetItem;
		nitem->setText(0, n->id);
		nitem->setText(1, n->type());

		QMap<QString,QVariant> prop = n->property;

		foreach(QString key, prop.keys())
		{
			QTreeWidgetItem * p = new QTreeWidgetItem;
			QVariant val = prop[key];
			p->setText(0, key);

			// Special values
			if(key == "taskType"){
				p->setText(1, TaskNames[val.toInt()]);
			}
			else if(key == "task"){
				Task * task = prop[key].value<Task*>();
				foreach(QString k, task->property.keys()){
					QVariant v = task->property[k];
					QString tval = v.toString();
					if(tval.size()){
						QTreeWidgetItem * taskItem = new QTreeWidgetItem;
						taskItem->setText(0, k);
						taskItem->setText(1, v.toString());
						p->addChild(taskItem);
					}
				}
			}
			else if(val.typeName()) // All other values
			{
				if(val.toString().size())
				{
					p->setText(1, val.toString());
				}
			}

			if(p->text(0).size()) 
				nitem->addChild(p);
			else
				delete p;
		}

		ui->nodesTree->addTopLevelItem(nitem);
	}
}

void GraphExplorer::fillEdgesInfo()
{
	foreach(Structure::Link * l, g->edges){
		QTreeWidgetItem * item = new QTreeWidgetItem;
		item->setText(0, l->id);
		item->setText(1, ""); // or l->type

		QMap<QString,QVariant> prop = l->property;

		foreach(QString key, prop.keys())
		{
			QTreeWidgetItem * p = new QTreeWidgetItem;
			QVariant val = prop[key];

			if(key == "") // Special values
			{
				p->setText(0, key);
				p->setText(1, "");
			}
			else if(val.typeName()) // All other values
			{
				if(val.toString().size())
				{
					p->setText(0, key);
					p->setText(1, val.toString());
				}
			}

			if(p->text(0).size()) 
				item->addChild(p);
			else
				delete p;
		}

		ui->edgesTree->addTopLevelItem(item);
	}
}

GraphExplorer::~GraphExplorer()
{
    delete ui;
}

void GraphExplorer::clear()
{
	ui->graphName->setText("Loading graph..");

	ui->nodesTree->clear();
	ui->edgesTree->clear();
}
