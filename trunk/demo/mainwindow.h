#pragma once

#include <QMainWindow>

#include "Session.h"
#include "Controls.h"
#include "ExporterWidget.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
    static DatasetMap getDataset(QString datasetPath = "dataset");

private:
    Ui::MainWindow *ui;
    QGLWidget * viewport;

    Scene * scene;
    Session * session;
    Controls * control;
	ExporterWidget * ewidget;

public slots:
	void message(QString m);
	void keyUpEvent(QKeyEvent* keyEvent);
	void showLogWindow();

	void prepareDemo();
};
