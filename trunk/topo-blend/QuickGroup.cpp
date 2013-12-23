#include "QuickGroup.h"
#include "ui_QuickGroup.h"

QuickGroup::QuickGroup(Structure::Graph * graph, QWidget *parent) : QDialog(parent), ui(new Ui::QuickGroup)
{
    ui->setupUi(this);
    this->g = graph;

    // Populate lists
	updateNodesList();
	updateCurrentGroups();

    // Connections
    this->connect(ui->list, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->groupButton, SIGNAL(clicked()), SLOT(doGrouping()));
	this->connect(ui->ungroupButton, SIGNAL(clicked()), SLOT(doUnGrouping()));
	this->connect(ui->curList, SIGNAL(itemSelectionChanged()), SLOT(groupSelected()));
	ui->list->connect(ui->clearButton, SIGNAL(clicked()), SLOT(clearSelection()));
}

QuickGroup::~QuickGroup()
{
    delete ui;
}

void QuickGroup::updateNodesList()
{
	ui->list->clear();

	foreach(Structure::Node * n, g->nodes) 
	{
		QListWidgetItem * item = new QListWidgetItem(n->id);
		ui->list->addItem(item);

		if(g->groupsOf(n->id).front().isEmpty()) item->setBackgroundColor(QColor(255,230,230));
	}
}

void QuickGroup::doGrouping()
{
	QList<QListWidgetItem *> items = ui->list->selectedItems();

	// Set black for all
	QVector<QString> nodes;
	foreach (QListWidgetItem * item, items)
		nodes.push_back( item->text() );

	g->addGroup(nodes);

	updateCurrentGroups();
	updateNodesList();
}

void QuickGroup::doUnGrouping()
{
	g->removeGroup( currentSelectedIndex() );

	updateCurrentGroups();
}

int QuickGroup::currentSelectedIndex()
{
	QModelIndexList indexes = ui->curList->selectionModel()->selectedIndexes();
	QVector<int> indexList;
	foreach(QModelIndex index, indexes){
		indexList.push_back(index.row());
	}

	if(indexList.isEmpty())
		return -1;
	else
		return indexList.front();
}

void QuickGroup::visualizeSelections()
{
    QList<QListWidgetItem *> items = ui->list->selectedItems();

    // Set black for all
    foreach (Structure::Node * node,  g->nodes)
    {
        node->vis_property["color"] = Qt::lightGray;
        node->vis_property["showControl"] = false;
    }

    // Set red for landmark
    foreach (QListWidgetItem * item, items)
        g->getNode(item->text())->vis_property["color"] = Qt::red;

    emit( updateView() );
}

void QuickGroup::updateCurrentGroups()
{
	int c = 0;

	// Display current groups
	ui->curList->clear();
	foreach(QVector<QString> group, g->groups)
	{
		QStringList nlist;
		foreach(QString nodeId, group) nlist << nodeId;
		QString groupID = QString("G%1").arg(c++);

		QListWidgetItem * newItem = new QListWidgetItem(groupID + ": " + nlist.join(", "));
		ui->curList->addItem( newItem );
	}

	emit( updateView() );
}

void QuickGroup::groupSelected()
{
	if(currentSelectedIndex() < 0) return;

	ui->list->clearSelection();
	
	foreach(QString nid, g->groups[currentSelectedIndex()])
	{
		foreach(QListWidgetItem* listItem, ui->list->findItems("*", Qt::MatchWildcard)){
			if(listItem->text() == nid)
				ui->list->setItemSelected(listItem, true);
		}
	}

	emit( updateView() );
}
