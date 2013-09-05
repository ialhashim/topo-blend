#include "GraphItem.h"

GraphItem::GraphItem(Structure::Graph *graph, QRectF region) : g(graph)
{
    setGeometry(region);
    connect(this, SIGNAL(geometryChanged()), SLOT(geometryHasChanged()));
}

void GraphItem::setGeometry(QRectF newGeometry)
{
    m_geometry = newGeometry;
    emit(geometryChanged());
}

void GraphItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option)
    Q_UNUSED(widget)

    painter->setBrush(QColor(0,0,0,0));
    //painter->drawRect(this->boundingRect());
}

void GraphItem::draw3D(qglviewer::Camera * camera)
{
    if(!g) return;

    // Save viewport
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

    // Set to graph rect
    QRectF r = boundingRect();
    Eigen::AlignedBox3d graphBBox = g->bbox();
    qglviewer::Vec vmin(graphBBox.min()), vmax(graphBBox.max());
    camera->fitBoundingBox(vmin,vmax);
    double d = graphBBox.diagonal().norm() * 0.4;
    camera->frame()->translate(camera->frame()->inverseTransformOf(qglviewer::Vec(0, 0, d)));
    camera->setScreenWidthAndHeight(r.width(), r.height());
    camera->loadProjectionMatrix();
    camera->loadModelViewMatrix();

    double y = -r.y() + (scene()->height() - r.height());
    glViewport(r.x(), y, r.width(), r.height());

    g->draw(0);

    // Restore
    glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
}

QRectF GraphItem::popState()
{
    if(states.isEmpty()) return boundingRect();
    return states.pop();
}

void GraphItem::pushState()
{
    states.push(boundingRect());
}

QPropertyAnimation *GraphItem::animateTo(QRectF newRect, int duration)
{
    QPropertyAnimation* anim = new QPropertyAnimation;
    anim->setTargetObject(this);
    anim->setPropertyName("geometry");
    anim->setStartValue(boundingRect());
    anim->setEndValue(newRect);
    anim->setDuration(duration);
    anim->setEasingCurve(QEasingCurve::OutQuad);
    return anim;
}

void GraphItem::geometryHasChanged()
{
    scene()->update();
}
