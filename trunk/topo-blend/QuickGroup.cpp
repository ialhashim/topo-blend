#include "QuickGroup.h"
#include "ui_QuickGroup.h"

QuickGroup::QuickGroup(Structure::Graph * graph, QWidget *parent) : QDialog(parent), ui(new Ui::QuickGroup)
{
    ui->setupUi(this);
    this->g = graph;

    // Populate lists
    foreach(Structure::Node * n, g->nodes) ui->list->addItem(new QListWidgetItem(n->id));
	updateCurrentGroups();

    // Connections
    this->connect(ui->list, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->groupButton, SIGNAL(clicked()), SLOT(doGrouping()));
	this->connect(ui->ungroupButton, SIGNAL(clicked()), SLOT(doUnGrouping()));
}

QuickGroup::~QuickGroup()
{
    delete ui;
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
		ui->curList->addItem(new QListWidgetItem(groupID + ": " + nlist.join(", ")));
	}

	emit( updateView() );
}
