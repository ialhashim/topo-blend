#pragma once

#include <QWidget>
#include <QStringList>
#include <QTreeWidget>

namespace Ui {class GraphExplorer;}

namespace Structure {struct Graph;}

struct SelectedItem{
	QStringList names;
	QString value;
};

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

	QStringList selectedNode();
	QStringList selectedEdge();
	QStringList treeSelection(QTreeWidget * tree);

	SelectedItem nodeSelection;
	SelectedItem edgeSelection;

	QStringList fullName(QTreeWidgetItem * item);

	QString dotPath;

public slots:
	void drawGraph();
	void drawGraphSVG();

	void filterNodes();
	void filterEdges();
	void filterTree(QTreeWidget * tree, QStringList filters, int column);
};
