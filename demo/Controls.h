#pragma once

#include "DemoGlobal.h"
#include <QWidget>
#include <QMap>

namespace Ui {class Controls;}

class Controls : public QWidget
{
    Q_OBJECT
    
public:
    explicit Controls(QWidget *parent = 0);
    ~Controls();

	void loadCategories( QString datasetPath );

public slots:
    void tabChanged(int index);
	void disableTabs();
	void enableTabs();

	void forceManualMatch();

signals:
    void hideAll();
    void showSelect();
    void showMatch();
    void showCreate();

	void categoriesLoaded(PropertyMap);

public:
    Ui::Controls *ui;
};
