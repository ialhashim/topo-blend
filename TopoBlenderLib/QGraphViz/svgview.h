#pragma once

#include <QGraphicsView>

QT_BEGIN_NAMESPACE
class QWheelEvent;
class QKeyEvent;
class QPaintEvent;
class QFile;
QT_END_NAMESPACE

class SvgView : public QGraphicsView
{
    Q_OBJECT

public:
    enum RendererType { Native, OpenGL };

    SvgView(QWidget *parent = 0);

    void openFile(const QFile &file);
    void setRenderer(RendererType type = OpenGL);

public slots:
    void setHighQualityAntialiasing(bool highQualityAntialiasing);

protected:
    void wheelEvent(QWheelEvent *event);
    void paintEvent(QPaintEvent *event);

	void keyReleaseEvent (QKeyEvent * event);

private:
    RendererType m_renderer;
    QGraphicsItem *m_svgItem;
};
