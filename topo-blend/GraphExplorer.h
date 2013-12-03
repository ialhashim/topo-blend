#pragma once

#include <QWidget>
#include <QStringList>
#include <QTreeWidget>

#include <QProcess>
#include "QGraphViz/svgview.h"
#include <QTemporaryFile>

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

	QProcess * p;
	SvgView * svgViewer;

private:
    Ui::GraphExplorer *ui;
	Structure::Graph * g;

	QMap<QString,QString> oldValues;
	void storeOldValues();

	void clear();
	void fillNodesInfo();
	void fillEdgesInfo();
	void fillGraphInfo();

	void fillInfoItem( QMap<QString,QVariant> prop, QTreeWidgetItem * item );

	QStringList selectedNode();
	QStringList selectedEdge();
	QStringList treeSelection(QTreeWidget * tree);

	SelectedItem nodeSelection;
	SelectedItem edgeSelection;

	QStringList fullName(QTreeWidgetItem * item);

	QString dotPath;

protected:
	virtual void hideEvent(QHideEvent *);

public slots:
	void drawGraph();
	void drawGraphSVG();

	void filterNodes();
	void filterEdges();
	void filterTree(QTreeWidget * tree, QStringList filters, int column);
};
