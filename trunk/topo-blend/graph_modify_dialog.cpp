#include "graph_modify_dialog.h"
#include "ui_graph_modify_dialog.h"

GraphModifyDialog::GraphModifyDialog(Structure::Graph * graph, QWidget *parent) : QDialog(parent), ui(new Ui::GraphModifyDialog)
{
    this->g = graph;

    ui->setupUi(this);

    this->connect( ui->linkButton, SIGNAL(clicked()), SLOT(link()) );
    this->connect( ui->unlinkButton, SIGNAL(clicked()), SLOT(unlink()) );
    this->connect( ui->removeButton, SIGNAL(clicked()), SLOT(remove()) );
	this->connect( ui->unlinkAllButton, SIGNAL(clicked()), SLOT(removeAll()) );

	this->connect(ui->list1, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));
	this->connect(ui->list2, SIGNAL(itemSelectionChanged()), SLOT(visualizeSelections()));

    // Populate lists
    foreach(Structure::Node * n, graph->nodes)
    {
        ui->list1->addItem(new QListWidgetItem(n->id));
        ui->list2->addItem(new QListWidgetItem(n->id));
    }
}

GraphModifyDialog::~GraphModifyDialog()
{
    delete ui;
}

void GraphModifyDialog::link()
{
    if(ui->list1->selectedItems().size() < 1) return;
    if(ui->list2->selectedItems().size() < 1) return;

    QString id1 = ui->list1->selectedItems().first()->text();
	foreach( QListWidgetItem *item, ui->list2->selectedItems())
	{
		QString id2 = item->text();
		g->addEdge(id1, id2);
	}

	emit (updateView());
}

void GraphModifyDialog::unlink()
{
    if(ui->list1->selectedItems().size() < 1) return;
    if(ui->list2->selectedItems().size() < 1) return;

    QString id1 = ui->list1->selectedItems().first()->text();

	foreach( QListWidgetItem *item, ui->list2->selectedItems())
	{
		QString id2 = item->text();
		g->removeEdge( g->getNode(id1), g->getNode(id2) );
	}

	emit( updateView() );
}

void GraphModifyDialog::remove()
{
    if(ui->list1->selectedItems().size() < 1) return;
    if(ui->list2->selectedItems().size() < 1) return;

    qDebug() << "Remove node: not coded!";
}

void GraphModifyDialog::visualizeSelections()
{
	QList<QListWidgetItem *> items_1 = ui->list1->selectedItems();
	QList<QListWidgetItem *> items_2 = ui->list2->selectedItems();

	// Set black for all
	foreach (Structure::Node * node,  g->nodes)
	{
		node->vis_property["color"] = Qt::lightGray;
		node->vis_property["showControl"] = false;
	}

	// Set red for landmark
	foreach (QListWidgetItem * item, items_1)
		g->getNode(item->text())->vis_property["color"] = Qt::red;

	foreach (QListWidgetItem * item, items_2)
		g->getNode(item->text())->vis_property["color"] = Qt::green;

	emit( updateView() );
}

void GraphModifyDialog::removeAll()
{
	foreach(Structure::Link * edge, g->edges)
		g->removeEdge(edge->n1, edge->n2);

	emit( updateView() );
}
