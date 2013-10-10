#pragma once

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsView>

class BlendPathWidget : public QWidget
{
    Q_OBJECT
public:
    explicit BlendPathWidget(int w, int h, QWidget *parent = 0);

	QGraphicsScene * scene;
	QGraphicsProxyWidget * proxy;

	void setBackgroundColor(QColor c);

	void buildScene(int w, int h);
private:
	QGraphicsView * view;

signals:
    
public slots:
};
