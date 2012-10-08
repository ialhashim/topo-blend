#include "topo_blend_widget.h"
#include "ui_topo_blend_widget.h"
#include "topo-blend.h"

topo_blend_widget::topo_blend_widget(topoblend * topo_blend, QWidget *parent) : QWidget(parent), ui(new Ui::topo_blend_widget)
{
    ui->setupUi(this);

    this->tb = topo_blend;
}

topo_blend_widget::~topo_blend_widget()
{
    delete ui;
}
