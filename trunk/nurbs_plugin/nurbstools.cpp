#include "nurbstools.h"
#include "ui_nurbstools.h"
#include "nurbs_plugin.h"

NURBSTools::NURBSTools(nurbs_plugin * usePlugin, QWidget *parent) : QWidget(parent), ui(new Ui::NURBSTools)
{
    ui->setupUi(this);
    this->plugin = usePlugin;

    // Connect
    plugin->connect(ui->fitCurveButton, SIGNAL(clicked()), SLOT(doFitCurve()));
    plugin->connect(ui->fitSurfaceButton, SIGNAL(clicked()), SLOT(doFitSurface()));

}

NURBSTools::~NURBSTools()
{
    delete ui;
}
