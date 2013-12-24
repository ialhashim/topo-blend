#pragma once

#include <QWidget>
#include <QAbstractButton>

namespace Ui {
class topo_blend_widget;
}

class topoblend;
class GraphCorresponder;
class Scheduler;
class TopoBlender;
class SynthesisManager;

class topo_blend_widget : public QWidget
{
    Q_OBJECT
    
public:
    explicit topo_blend_widget(topoblend * topo_blend, QWidget *parent = 0);
    ~topo_blend_widget();

	int synthesisSamplesCount();
	
	QWidget * simpleWidget();

	void addTab( QWidget * widget );

public slots:
    void renderViewer();
    void renderAnimation();
    void loadAnimationModel();
    void doBlend();
	void loadCorrespondenceModel();
	void showGroupingDialog();
	void vizButtonClicked(QAbstractButton* b);
	void toggleCheckOption(QString optionName);
	void setCheckOption( QString optionName, bool toValue = true );
	void setSynthSamplesCount( int );
	void generateSynthesisData();

	// Model manipulation
	void normalizeModel();
	void bottomCenterModel();
	void moveModel();
	void rotateModel();
	void scaleModel();
	void exportAsOBJ();

	// Part manipulation
	void reverseCurve();
	void updatePartsList();
	void updateVisualization();

	void splatSizeChanged(double newSize);

	// Jobs
	void saveJob();
	void loadJob();
	void loadJobFile(QString job_filename);
    QString loadJobFileName();
	void saveJob(GraphCorresponder * gcoor, Scheduler * scheduler, TopoBlender * blender, SynthesisManager * s_manager);

private:
	Ui::topo_blend_widget *ui;
    topoblend * tb;
};
