#include "graph_modify_dialog.h"
#include "ui_graph_modify_dialog.h"
#include <QInputDialog>

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

	this->connect(ui->updateLinkButton, SIGNAL(clicked()), SLOT(updateLink()) );

	this->connect(ui->renameButton, SIGNAL(clicked()), SLOT(renameNodes()) );

	g->property["showNames"] = true;

	updateLists();
}

GraphModifyDialog::~GraphModifyDialog()
{
	g->property["showNames"] = false;

    delete ui;
}

void GraphModifyDialog::renameNodes()
{
	foreach( QListWidgetItem *item, ui->list2->selectedItems())
	{
		QString nid = item->text();
		bool ok;
		QString newID = QInputDialog::getText(0,tr("Node name"),tr("User name:"),QLineEdit::Normal,nid, &ok);
		if (ok && !newID.isEmpty())
			g->renameNode(nid, newID);
	}

	updateLists();

	emit ( updateView() );
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

	emit ( updateView() );
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

void GraphModifyDialog::updateLink()
{
	if(ui->list1->selectedItems().size() < 1) return;
	if(ui->list2->selectedItems().size() < 1) return;

	QString id1 = ui->list1->selectedItems().last()->text();
	QString id2 = ui->list2->selectedItems().last()->text();

	Structure::Link * l = g->getEdge(id1,id2); if(!l) return;

	Vec4d coord1(ui->uA->value(), ui->vA->value(), 0, 0);
	Vec4d coord2(ui->uB->value(), ui->vB->value(), 0, 0);

	l->setCoord(id1, std::vector<Vec4d>(1,coord1));
	l->setCoord(id2, std::vector<Vec4d>(1,coord2));

	emit( updateView() );
}

void GraphModifyDialog::remove()
{
    if(ui->list1->selectedItems().size() < 1) return;

	foreach( QListWidgetItem *item, ui->list1->selectedItems())
	{
		QString id = item->text();
		g->removeNode( id );
		qDebug() << "Removed node: " << id;
	}

	ui->list1->clear();
	ui->list2->clear();

	updateLists();

	emit( updateView() );
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

	// Edge coordinates if it exists
	{
		if(ui->list1->selectedItems().size() < 1) return;
		if(ui->list2->selectedItems().size() < 1) return;

		QString id1 = ui->list1->selectedItems().last()->text();
		QString id2 = ui->list2->selectedItems().last()->text();

		Structure::Link * l = g->getEdge(id1,id2); if(!l) return;

		Vec4d coord1 = l->getCoord(id1).front();
		Vec4d coord2 = l->getCoord(id2).front();

		ui->uA->setValue(coord1[0]); 
		ui->vA->setValue(coord1[1]);

		ui->uB->setValue(coord2[0]); 
		ui->vB->setValue(coord2[1]);
	}
}

void GraphModifyDialog::removeAll()
{
	foreach(Structure::Link * edge, g->edges)
		g->removeEdge(edge->n1, edge->n2);

	emit( updateView() );
}

void GraphModifyDialog::updateLists()
{
	ui->list1->clear();
	ui->list2->clear();

	// Populate lists
	foreach(Structure::Node * n, g->nodes)
	{
		ui->list1->addItem(new QListWidgetItem(n->id));
		ui->list2->addItem(new QListWidgetItem(n->id));
	}
}
