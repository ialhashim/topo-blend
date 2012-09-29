#ifndef RESAMPLEWIDGET_H
#define RESAMPLEWIDGET_H

#include <QDialog>

class myresample;

namespace Ui {
class ResampleWidget;
}

class ResampleWidget : public QDialog
{
    Q_OBJECT
    
public:
    explicit ResampleWidget(myresample *, QWidget *parent = 0);
    ~ResampleWidget();

private:
    Ui::ResampleWidget *ui;
    myresample * r;

};

#endif // RESAMPLEWIDGET_H
