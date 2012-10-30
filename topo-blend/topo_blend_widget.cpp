#include "topo-blend.h"
#include "topo_blend_widget.h"
#include "ui_topo_blend_widget.h"
#include "ui_animationWidget.h"

#include "QuickMesh.h"
#include "QuickViewer.h"
#include <QFileDialog>

QuickViewer * viewer = NULL;
QString cur_filename;

topo_blend_widget::topo_blend_widget(topoblend * topo_blend, QWidget *parent) : QWidget(parent), ui(new Ui::topo_blend_widget)
{
    ui->setupUi(this);

    this->tb = topo_blend;

    topo_blend->connect(ui->button1, SIGNAL(clicked()), SLOT(generateTwoChairModels()));
    topo_blend->connect(ui->button2, SIGNAL(clicked()), SLOT(loadModel()));
	topo_blend->connect(ui->button3, SIGNAL(clicked()), SLOT(doBlend()));
	topo_blend->connect(ui->button4, SIGNAL(clicked()), SLOT(experiment1()));
	topo_blend->connect(ui->button5, SIGNAL(clicked()), SLOT(generateTwoSimpleModels()));
    this->connect(ui->button6, SIGNAL(clicked()), SLOT(renderViewer()));
}

topo_blend_widget::~topo_blend_widget()
{
    delete ui;
}

void topo_blend_widget::renderViewer()
{
    QDialog * d = new QDialog;
    d->show();

    Ui::AnimationForm aniForm;
    aniForm.setupUi(d);

    viewer = new QuickViewer();
    aniForm.mainLayout->addWidget(viewer);
    viewer->makeCurrent();

    this->connect(aniForm.button, SIGNAL(clicked()), SLOT(renderAnimation()));
    this->connect(aniForm.loadButton, SIGNAL(clicked()), SLOT(loadAnimationModel()));
}

void topo_blend_widget::renderAnimation()
{
    QString filename = cur_filename;
    filename = filename.replace("0","%1");

    for(int i = 0; i < 12; i++)
    {
        viewer->m->load(filename.arg(i));
        viewer->updateGL();
        viewer->saveSnapshot(filename.arg(i) + ".png");
    }
}

void topo_blend_widget::loadAnimationModel()
{
    viewer->m = new QuickMesh;
    cur_filename = QFileDialog::getOpenFileName(this, tr("Open Mesh"), "", tr("Mesh Files (*.obj *.off)"));
    viewer->m->load(cur_filename);
}
