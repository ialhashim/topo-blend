#include <QtConcurrentRun>
#include <QSet>
#include <QStack>
#include <QFileDialog>
#include <QProcess>
#include <QTemporaryFile>

#include "GraphExplorer.h"
#include "ui_GraphExplorer.h"

#include "StructureGraph.h"
#include "Scheduler.h"
#include "Task.h"

#include "ExportDynamicGraph.h"
#include "QGraphViz/svgview.h"

QProcess * p = NULL;
SvgView * svgViewer = NULL;

#ifndef popen
#define popen _popen
#define pclose _pclose
#endif

static inline std::string exec(char* cmd) {
	FILE* pipe = popen(cmd, "r");
	if (!pipe) return "ERROR";
	char buffer[128];
	std::string result = "";
	while(!feof(pipe)) {
		if(fgets(buffer, 128, pipe) != NULL)
			result += buffer;
	}
	pclose(pipe);
	return result;
}

GraphExplorer::GraphExplorer(QWidget *parent): QWidget(parent), ui(new Ui::GraphExplorer)
{
	//setAttribute(Qt::WA_DeleteOnClose);
	ui->setupUi(this);

	g = NULL;
	clear();

	this->connect(ui->nodesFilterProperty, SIGNAL(textChanged(QString)), SLOT(filterNodes()));
	this->connect(ui->nodesFilterValue, SIGNAL(textChanged(QString)), SLOT(filterNodes()));

	this->connect(ui->edgesFilterProperty, SIGNAL(textChanged(QString)), SLOT(filterEdges()));
	this->connect(ui->edgesFilterValue, SIGNAL(textChanged(QString)), SLOT(filterEdges()));

	ui->nodesTree->sortByColumn(0);
	ui->edgesTree->sortByColumn(0);
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

	// Apply filters
	filterNodes();
	filterEdges();

	if(!this->isVisible()) this->show();
}

void GraphExplorer::drawGraph()
{
	// Create graph visualization
	/*QString tmpGraphFilename = "tempGraph_"+QString::number(QDateTime::currentMSecsSinceEpoch());
	visualizeStructureGraph(g, tmpGraphFilename, "");
	QImage gimg(tmpGraphFilename+".png");
	QFile::remove(tmpGraphFilename+".png");
	QFile::remove(tmpGraphFilename+".gv");
	// Set graph visualization
	//ui->graphImage->setPixmap( QPixmap::fromImage(gimg.scaled(gimg.size() * 0.5, Qt::KeepAspectRatio, Qt::SmoothTransformation)) );*/

	if(!dotPath.size()){
		#ifdef Q_OS_WIN
		dotPath = "\"" + QString(exec("where dot").c_str()).replace("\\","/").trimmed() + "\"";
		#else
		dotPath = QString(exec("which dot").c_str());
		#endif

		if(!dotPath.size())
		{
			// Ask user for help
			dotPath = QFileDialog::getOpenFileName(0, "Graphviz dot application", "dot", tr("Application (*.*)"));
			if(dotPath == "") dotPath = "-";
		}
	}
	if(dotPath == "-") return;

	p = new QProcess;
	this->connect(p, SIGNAL(finished(int, QProcess::ExitStatus)), SLOT(drawGraphSVG()));
	p->start( dotPath, QStringList() << "-Tsvg" );
	if (!p->waitForStarted()){
		qDebug() << "Error process: " << p->error();
		return;
	}

	p->write( qPrintable(toGraphvizFormat(g,"","")) );
	p->closeWriteChannel();
	p->waitForFinished(-1);

	double x = 0;
}

void GraphExplorer::drawGraphSVG()
{
	QString svgData = p->readAllStandardOutput();
	QTemporaryFile file;
	if (file.open()){
		QTextStream out(&file);
		out << svgData;
		file.close();

		if(!svgViewer) 
		{
			svgViewer = new SvgView;
			ui->graphVizLayout->addWidget(svgViewer);
			ui->graphImage->setText("");
		}

		svgViewer->openFile(file);
	}
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

void GraphExplorer::filterNodes()
{
	ui->nodesTree->clear();
	fillNodesInfo();
	filterTree(ui->nodesTree, ui->nodesFilterProperty->text().split("|",QString::SkipEmptyParts), 0);
	filterTree(ui->nodesTree, ui->nodesFilterValue->text().split("|",QString::SkipEmptyParts), 1);
}

void GraphExplorer::filterEdges()
{
	ui->edgesTree->clear();
	fillEdgesInfo();
	filterTree(ui->edgesTree, ui->edgesFilterProperty->text().split("|",QString::SkipEmptyParts), 0);
	filterTree(ui->edgesTree, ui->edgesFilterValue->text().split("|",QString::SkipEmptyParts), 1);
}

void GraphExplorer::filterTree(QTreeWidget * tree, QStringList filters, int column)
{
	if(!filters.size()) return;

	QSet<QTreeWidgetItem*> terminals;

	// Get all terminals
	QTreeWidgetItemIterator it(tree);
	while (*it) {
		if((*it)->childCount() == 0) terminals.insert(*it);
		it++;
	}

	// Check if terminals name is within filtered
	QVector<QTreeWidgetItem*> keep;
	foreach(QTreeWidgetItem * item, terminals)
	{
		QStringList itemName = column == 0 ? fullName(item) : (QStringList() << item->text(column));

		bool isKeep = false;

		foreach(QString n, itemName){
			foreach(QString f, filters){
				if(n.contains(f, Qt::CaseInsensitive)){
					isKeep = true;
					break;
				}
			}
			if(isKeep) break;
		}

		if(isKeep)
		{
			QTreeWidgetItem * filtered = new QTreeWidgetItem();
			filtered->setText(0, fullName(item).join(" / "));
			filtered->setText(1, item->text(1));
			keep.push_back(filtered);
		}
	}

	tree->clear();

	foreach(QTreeWidgetItem* item, keep){
		tree->addTopLevelItem(item);
	}
}

QStringList GraphExplorer::selectedNode()
{
	return treeSelection(ui->nodesTree);
}

QStringList GraphExplorer::selectedEdge()
{
	return treeSelection(ui->edgesTree);
}

QStringList GraphExplorer::treeSelection( QTreeWidget * tree )
{
	QStringList selection;

	foreach(QTreeWidgetItem * item, tree->selectedItems()){
		QTreeWidgetItem * parent = item;

		while( parent ){
			selection << parent->text(0);
			parent = parent->parent();
		}
	}

	return selection;
}

QStringList GraphExplorer::fullName( QTreeWidgetItem * item )
{
	QStringList nameList;
	QTreeWidgetItem * parent = item;
	while( parent ){
		nameList << parent->text(0);
		parent = parent->parent();
	}
	return nameList;
}
