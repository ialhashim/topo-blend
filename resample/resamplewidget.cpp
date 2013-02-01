#include "resamplewidget.h"
#include "ui_resamplewidget.h"
#include "resample.h"

ResampleWidget::ResampleWidget(myresample *resampler, QWidget *parent): QDialog(parent), ui(new Ui::ResampleWidget)
{
    ui->setupUi(this);

    /// Stay on top
    setWindowFlags(Qt::WindowStaysOnTopHint);

    this->r = resampler;

    connect(ui->resampleButton, SIGNAL(clicked()), (const QObject*) resampler, SLOT(doResample()));
    connect(ui->parameterizeButton, SIGNAL(clicked()), (const QObject*) resampler, SLOT(doParameterize()));
}

ResampleWidget::~ResampleWidget()
{
    delete ui;
}
