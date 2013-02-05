#ifndef NURBSTOOLS_H
#define NURBSTOOLS_H

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

private:
    Ui::NURBSTools *ui;
};

#endif // NURBSTOOLS_H
