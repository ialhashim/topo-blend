#include "GraphItem.h"

GraphItem::GraphItem(Structure::Graph *graph, QRectF region, qglviewer::Camera * camera) : g(graph), camera(camera)
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

QRectF GraphItem::setCamera()
{
	if(camera->type() != qglviewer::Camera::PERSPECTIVE) camera->setType(qglviewer::Camera::PERSPECTIVE);

	QRectF r = boundingRect();

	qglviewer::Vec viewDir = camera->viewDirection();

	Eigen::AlignedBox3d graphBBox = g->cached_bbox();

	double distance = graphBBox.diagonal().size() * 0.9;
	Vector3 center = graphBBox.center();
	Vector3 newPos = center - (distance * Vector3(viewDir[0], viewDir[1], viewDir[2]));

	camera->setRevolveAroundPoint( qglviewer::Vec(center) );
	camera->frame()->setPositionWithConstraint(qglviewer::Vec(newPos));

	camera->setScreenWidthAndHeight(r.width(), r.height());
	camera->loadProjectionMatrix();
	camera->loadModelViewMatrix();

	qglviewer::Vec cp = camera->position();
	qDebug() << cp[0] << "," << cp[1] << "," << cp[2];

	return r;
}

void GraphItem::draw3D()
{
    if(!g) return;

    // Save viewport
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);

	// Set to graph rect
	QRectF r = setCamera();

	double y = -r.y() + (scene()->height() - r.height());
	glViewport(r.x(), y, r.width(), r.height());

    g->draw();

	// DEBUG:
	ps1.draw(); ps2.draw();	vs1.draw(3); vs2.draw();	ss1.draw(); ss2.draw();

    // Restore
    glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
}

void GraphItem::pick( int x, int y )
{
	if(!m_geometry.contains(x,y)) return;

	qglviewer::Vec orig, dir;
	QMap<double,Vector3> isects;
	QMap<double,QString> isectNode;

	x -= m_geometry.x();
	y -= m_geometry.y();

	setCamera();
	camera->convertClickToLine(QPoint(x,y), orig, dir);
	
	foreach(Structure::Node * n, g->nodes)
	{
		SurfaceMesh::Model* nodeMesh = n->property["mesh"].value<SurfaceMesh::Model*>();
		if(!nodeMesh) continue;

		Vector3VertexProperty points = nodeMesh->vertex_property<Vector3>(VPOINT);

		foreach(Face f, nodeMesh->faces())
		{
			std::vector<Vector3> tri;
			Surface_mesh::Vertex_around_face_circulator vit, vend;
			vit = vend = nodeMesh->vertices(f);
			do{ tri.push_back( points[vit] ); } while(++vit != vend);

			Vector3 origin (orig[0], orig[1], orig[2]);
			Vector3 direction (dir[0], dir[1], dir[2]);
			Vector3 ipoint;

			if( intersectRayTri(tri, origin, direction, ipoint) )
			{
				double dist = (origin - ipoint).norm();
				isects[dist] = ipoint;
				isectNode[dist] = n->id;
			}
		}
	}

	// Only consider closest hit
	if(isects.size())
	{
		double minDist = isects.keys().front();
		Vector3 ipoint = isects[minDist];
		Vector3 direction(-dir[0],-dir[1],-dir[2]);

		direction *= 0.1;

		ss1.addSphere(ipoint, 0.03f);
		vs1.addVector(ipoint, direction);

		emit( hit(this, isectNode[minDist], ipoint) );
	}
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
