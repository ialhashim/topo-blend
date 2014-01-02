#include "svgview.h"

#include <QFile>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QGraphicsRectItem>
#include <QGraphicsSvgItem>
#include <QPaintEvent>
#include <qmath.h>
#include <QGLWidget>
#include <QTextStream>
#include <QFileDialog>

QString svgFile;

// Usage e.g.:     setCentralWidget(new SvgView);
SvgView::SvgView(QWidget *parent) : QGraphicsView(parent), m_renderer(OpenGL), m_svgItem(0)
{
    setScene(new QGraphicsScene(this));
    setTransformationAnchor(AnchorUnderMouse);
    setDragMode(ScrollHandDrag);
    setViewportUpdateMode(FullViewportUpdate);

    setRenderer(m_renderer);
    setHighQualityAntialiasing(true);
}

void SvgView::openFile(const QFile &file)
{
    if (!file.exists())
        return;

    QGraphicsScene *s = scene();
    s->clear();
    //resetTransform();

    m_svgItem = new QGraphicsSvgItem(file.fileName());
    m_svgItem->setFlags(QGraphicsItem::ItemClipsToShape);
    m_svgItem->setCacheMode(QGraphicsItem::NoCache);
    m_svgItem->setZValue(0);
    m_svgItem->setFlag(QGraphicsItem::ItemIsMovable, true);

    s->addItem(m_svgItem);

	QFile ff(file.fileName());
	ff.open(QFile::ReadOnly | QFile::Text);
	QTextStream in(&ff);
	svgFile = in.readAll();
	ff.close();
}

void SvgView::setRenderer(RendererType type)
{
    m_renderer = type;
    setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
}

void SvgView::setHighQualityAntialiasing(bool highQualityAntialiasing)
{
    setRenderHint(QPainter::HighQualityAntialiasing, highQualityAntialiasing);
}

void SvgView::paintEvent(QPaintEvent *event)
{
    QGraphicsView::paintEvent(event);
}

void SvgView::wheelEvent(QWheelEvent *event)
{
    qreal factor = qPow(1.2, event->delta() / 240.0);
    scale(factor, factor);
    event->accept();
}

void SvgView::keyReleaseEvent( QKeyEvent * event )
{
	if(event->key() == Qt::Key_Space)
	{
		QString svgFileName = QFileDialog::getSaveFileName(0, "Save graph SVG", "", tr("SVG Files (*.SVG)"));

		QFile ff(svgFileName);
		ff.open(QFile::WriteOnly | QFile::Text);
		QTextStream out(&ff);
		out << svgFile;
		ff.close();
	}
}

