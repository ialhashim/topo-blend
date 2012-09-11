#include "resamplewidget.h"
#include "ui_resamplewidget.h"

ResampleWidget::ResampleWidget(myresample *resampler, QWidget *parent): QDialog(parent), ui(new Ui::ResampleWidget)
{
    ui->setupUi(this);

    /// Stay on top
    setWindowFlags(Qt::WindowStaysOnTopHint);

    this->r = resampler;

    connect(ui->resampleButton, SIGNAL(clicked()), (const QObject*) resampler, SLOT(doResample()));
}

ResampleWidget::~ResampleWidget()
{
    delete ui;
}
