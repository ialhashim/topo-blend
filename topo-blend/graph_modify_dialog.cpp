#include "graph_modify_dialog.h"
#include "ui_graph_modify_dialog.h"

GraphModifyDialog::GraphModifyDialog(Structure::Graph * graph, QWidget *parent) : QDialog(parent), ui(new Ui::GraphModifyDialog)
{
    this->g = graph;

    ui->setupUi(this);

    this->connect( ui->linkButton, SIGNAL(clicked()), SLOT(link()) );
    this->connect( ui->unlinkButton, SIGNAL(clicked()), SLOT(unlink()) );
    this->connect( ui->removeButton, SIGNAL(clicked()), SLOT(remove()) );

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
    QString id2 = ui->list2->selectedItems().first()->text();

    g->addEdge(id1, id2);
}

void GraphModifyDialog::unlink()
{
    if(ui->list1->selectedItems().size() < 1) return;
    if(ui->list2->selectedItems().size() < 1) return;

    QString id1 = ui->list1->selectedItems().first()->text();
    QString id2 = ui->list2->selectedItems().first()->text();

    g->removeEdge( g->getNode(id1), g->getNode(id2) );
}

void GraphModifyDialog::remove()
{
    if(ui->list1->selectedItems().size() < 1) return;
    if(ui->list2->selectedItems().size() < 1) return;

    qDebug() << "Remove node: not coded!";
}
