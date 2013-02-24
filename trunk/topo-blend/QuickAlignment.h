#pragma once

#include <QDialog>
#include "StructureGraph.h"

namespace Ui {
class QuickAlignment;
}

class QuickAlignment : public QDialog
{
    Q_OBJECT
    
public:
    explicit QuickAlignment(Structure::Graph * graphA, Structure::Graph * graphB, QWidget *parent = 0);
    ~QuickAlignment();
    
private:
    Ui::QuickAlignment *ui;
    Structure::Graph *ga, *gb;

	static int quadrant(Vector3 v);

public slots:
	void doPartAlignment();
	
signals:
    void updateView();

};
