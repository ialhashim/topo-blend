#include "GraphModifyWidget.h"
#include "ui_GraphModifyWidget.h"
#include <QInputDialog>

GraphModifyWidget::GraphModifyWidget(Structure::Graph * graph, QWidget *parent) : QDialog(parent), ui(new Ui::GraphModifyWidget)
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

	updateLists();
}

GraphModifyWidget::~GraphModifyWidget()
{
	g->property["showNames"] = false;

    delete ui;
}

void GraphModifyWidget::renameNodes()
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

void GraphModifyWidget::link()
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

void GraphModifyWidget::unlink()
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

void GraphModifyWidget::updateLink()
{
	if(ui->list1->selectedItems().size() < 1) return;
	if(ui->list2->selectedItems().size() < 1) return;

	QString id1 = ui->list1->selectedItems().last()->text();
	QString id2 = ui->list2->selectedItems().last()->text();

	Structure::Link * l = g->getEdge(id1,id2); if(!l) return;

	Vector4d coord1(ui->uA->value(), ui->vA->value(), 0, 0);
	Vector4d coord2(ui->uB->value(), ui->vB->value(), 0, 0);

	l->setCoord(id1, Array1D_Vector4d(1,coord1));
	l->setCoord(id2, Array1D_Vector4d(1,coord2));

	emit( updateView() );
}

void GraphModifyWidget::remove()
{
    if(ui->list2->selectedItems().size() < 1) return;

	foreach( QListWidgetItem *item, ui->list2->selectedItems())
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

void GraphModifyWidget::visualizeSelections()
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

		Vector4d coord1 = l->getCoord(id1).front();
		Vector4d coord2 = l->getCoord(id2).front();

		ui->uA->setValue(coord1[0]); 
		ui->vA->setValue(coord1[1]);

		ui->uB->setValue(coord2[0]); 
		ui->vB->setValue(coord2[1]);
	}
}

void GraphModifyWidget::removeAll()
{
	foreach(Structure::Link * edge, g->edges)
		g->removeEdge(edge->n1, edge->n2);

	emit( updateView() );
}

void GraphModifyWidget::updateLists()
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
