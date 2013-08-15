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
	double remeshParamter();
	double voxelParamter();

private:
    Ui::NURBSTools *ui;
};
