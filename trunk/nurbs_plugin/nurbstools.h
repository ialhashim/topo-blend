#pragma once

#include <QWidget>

namespace Ui {
class NURBSTools;
}

class nurbs_plugin;

class NURBSTools : public QWidget
{
    Q_OBJECT
    
public:
    explicit NURBSTools(nurbs_plugin * usePlugin, QWidget *parent = 0);
    ~NURBSTools();
    
    nurbs_plugin * plugin;

	int uCount();
	int vCount();

	double resolution();

	int contractIterations();

	bool isRemesh();
	bool isVoxelize();
	bool isUseMedial();
	bool isProtectSmallFeatures();

	double remeshParamter();
	double voxelParamter();

public slots:
	void selectionChanged();
	void fillList();
	QStringList selectedGroups();

	void convertToCurve();
	void convertToSheet();
private:
    Ui::NURBSTools *ui;
};
