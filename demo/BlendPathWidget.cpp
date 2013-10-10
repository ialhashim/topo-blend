#include <QDebug>
#include <QHBoxLayout>
#include <QGraphicsRectItem>
#include "BlendPathWidget.h"

BlendPathWidget::BlendPathWidget(int w, int h, QWidget *parent) : QWidget(parent), scene(NULL), view(NULL)
{
    this->setFixedSize(w,h);
    this->setWindowFlags(Qt::FramelessWindowHint);
	this->setMouseTracking(true);

	QHBoxLayout * m_layout = new QHBoxLayout;
	m_layout->setSpacing(0);
	m_layout->setMargin(0);
	this->setLayout(m_layout);

	buildScene(w, h);
}

void BlendPathWidget::buildScene(int w, int h)
{
	if(view) view->deleteLater();
	if(scene) scene->deleteLater();

	scene = new QGraphicsScene;
	view = new QGraphicsView(scene);
	view->setMouseTracking(true);
	view->setResizeAnchor(QGraphicsView::NoAnchor);
	view->setAlignment(Qt::AlignLeft | Qt::AlignTop);
	view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	view->setRenderHint(QPainter::HighQualityAntialiasing, true);
	view->setRenderHint(QPainter::SmoothPixmapTransform, true);

	QGraphicsRectItem * dummy = scene->addRect(0,0,w,h);
	dummy->setVisible(false);

	QString viewStyle = "	QGraphicsView { background-color: transparent; } \
						QScrollBar {border: 0px solid black; \
						height: 10px; \
						background-color: transparent; } \
						QScrollBar::add-page, QScrollBar::sub-page { background-color: transparent; } \
						QScrollBar::add-line, QScrollBar::sub-line, QScrollBar::handle { border: 1px solid rgba(0,0,0,200); }";
	view->setStyleSheet(viewStyle);

	layout()->addWidget(view);
}

void BlendPathWidget::setBackgroundColor( QColor c )
{
	QString widgetBackground = "background:" + QString(" rgba(%1,%2,%3,%4) ").arg(
		c.red()).arg(c.green()).arg(c.blue()).arg(c.alpha());

	QString widgetBorder = "border: 0px solid black";

	setStyleSheet( QString("QWidget { %1; %2; }").arg(widgetBackground).arg(widgetBorder) );
}
