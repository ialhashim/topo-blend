#pragma once

#include <QDialog>
#include "StructureGraph.h"

namespace Ui {
class GraphModifyWidget;
}

class GraphModifyWidget : public QDialog
{
    Q_OBJECT
    
public:
    explicit GraphModifyWidget(Structure::Graph * graph, QWidget *parent = 0);
    ~GraphModifyWidget();
    
private:
    Ui::GraphModifyWidget *ui;
    Structure::Graph * g;

public slots:
    void link();
    void unlink();
	void updateLink();
    void remove();
	void removeAll();
	void visualizeSelections();

	void updateLists();
	void renameNodes();

signals:
	void updateView();
};

