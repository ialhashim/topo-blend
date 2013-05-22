#pragma once

#include <QDialog>
#include "StructureGraph.h"

namespace Ui {
class QuickGroup;
}

class QuickGroup : public QDialog
{
    Q_OBJECT
    
public:
    explicit QuickGroup(Structure::Graph * graph, QWidget *parent = 0);
    ~QuickGroup();
    
private:
    Ui::QuickGroup *ui;
    Structure::Graph *g;

public slots:
    void doGrouping();
	void doUnGrouping();
    void visualizeSelections();
	void updateCurrentGroups();
	int currentSelectedIndex();

signals:
    void updateView();
};
