#pragma once

#include <QWidget>

namespace Ui {class GraphExplorer;}

namespace Structure {struct Graph;}

class GraphExplorer : public QWidget
{
    Q_OBJECT
    
public:
    explicit GraphExplorer(QWidget *parent = 0);
    ~GraphExplorer();

	void update(Structure::Graph * graph);

private:
    Ui::GraphExplorer *ui;
	Structure::Graph * g;

	void clear();
	void fillNodesInfo();
	void fillEdgesInfo();

public slots:
	void drawGraph();
};
