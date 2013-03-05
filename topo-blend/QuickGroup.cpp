#include "QuickGroup.h"
#include "ui_QuickGroup.h"

QuickGroup::QuickGroup(Structure::Graph * graph, QWidget *parent) : QDialog(parent), ui(new Ui::QuickGroup)
{
    ui->setupUi(this);
    this->g = graph;

    // Populate list
    foreach(Structure::Node * n, g->nodes) ui->list->addItem(new QListWidgetItem(n->id));

    // Connections
    this->connect(ui->list, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->groupButton, SIGNAL(clicked()), SLOT(doGrouping()));
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
	 
    emit( updateView() );
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
