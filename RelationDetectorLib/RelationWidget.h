#pragma once

#include <QWidget>
#include "RelationManager.h"

namespace Ui {
class RelationWidget;
}

class RelationWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit RelationWidget(topoblend * tb, QWidget *parent = 0);
    ~RelationWidget();
    
    RelationManager * r_manager;

private:
    Ui::RelationWidget *ui;
};
