#pragma once

#include <QWidget>
#include <QLabel>
namespace Ui { class Wizard; }

#include "QtAwesome/QtAwesome.h"
extern QtAwesome* awesome;

#include "topo-blend.h"

class Wizard : public QWidget
{
    Q_OBJECT
    
public:
    explicit Wizard(topoblend *parent, QWidget * parentWidget);
    ~Wizard();

	QVector<QLabel*> icons;
	QVector<QLayout*> layouts;
	
	void setOpacity( QString objName, double toValue );

	topoblend *tb;

public slots:
	void loadShapeA();
	void loadShapeB();
	void matchingButton();
	void generateBlend();

private:
    Ui::Wizard *ui;
};
