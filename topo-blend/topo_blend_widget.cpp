#include "topo-blend.h"
#include "topo_blend_widget.h"
#include "ui_topo_blend_widget.h"

topo_blend_widget::topo_blend_widget(topoblend * topo_blend, QWidget *parent) : QWidget(parent), ui(new Ui::topo_blend_widget)
{
    ui->setupUi(this);

    this->tb = topo_blend;

    topo_blend->connect(ui->button1, SIGNAL(clicked()), SLOT(generateTwoModels()));
    topo_blend->connect(ui->button2, SIGNAL(clicked()), SLOT(loadModel()));
}

topo_blend_widget::~topo_blend_widget()
{
    delete ui;
}
