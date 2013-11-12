#include "GraphItem.h"

#include <Eigen/Geometry>

GraphItem::GraphItem(Structure::Graph *graph, QRectF region, qglviewer::Camera * camera) : g(graph), camera(camera)
{
    setGeometry(region);
    connect(this, SIGNAL(geometryChanged()), SLOT(geometryHasChanged()));
}

GraphItem::~GraphItem()
{
	delete g;
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

	painter->setPen(QPen(Qt::red));
    
	// DEBUG:
	//painter->drawRect(this->boundingRect());
	//painter->drawText(m_geometry.x() + 10, m_geometry.y() + 20, this->name);
}

QRectF GraphItem::setCamera()
{
	if(camera->type() != qglviewer::Camera::PERSPECTIVE) camera->setType(qglviewer::Camera::PERSPECTIVE);

	QRectF r = boundingRect();

	qglviewer::Vec viewDir = camera->viewDirection();

	Eigen::AlignedBox3d graphBBox = g->cached_bbox();

	double distance = graphBBox.sizes().maxCoeff() * 2.5;
	Vector3 center = graphBBox.center();
	Vector3 newPos = center - (distance * Vector3(viewDir[0], viewDir[1], viewDir[2]));

	camera->setRevolveAroundPoint( qglviewer::Vec(center) );

    qglviewer::Vec new_pos(newPos);
    camera->frame()->setPositionWithConstraint(new_pos);

	camera->setScreenWidthAndHeight(r.width(), r.height());
	camera->loadProjectionMatrix();
	camera->loadModelViewMatrix();

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

	// Draw strokes
	{
		glDisable(GL_LIGHTING);

		QVector< QVector<Vector3> > allStorkes = g->property["strokes"].value< QVector< QVector<Vector3> > >();

		foreach(QVector<Vector3> skeleton, allStorkes)
		{
			int N = skeleton.size();

			if(N > 2 && g->property.contains("strokeColor"))
			{
				// Draw skeleton
				if(false)
				{
					glColorQt(QColor(0,0,0));
					glLineWidth(40);
					glEnableClientState(GL_VERTEX_ARRAY);
					glVertexPointer(3, GL_DOUBLE, 0, &skeleton[0][0]);
					glDrawArrays(GL_LINE_STRIP, 0, skeleton.size());
					glDisableClientState(GL_VERTEX_ARRAY);
				}

				QColor curColor = g->property["strokeColor"].value<QColor>();
				glColorQt(curColor);

				// Build a connected tube from cross-sections
				{
					QVector< QVector<Vector3> > crossSections;
					QVector<Vector3> crossSection;

					Vector3 t0 = (skeleton[1] - skeleton[0]).normalized();
					Vector3 start = orthogonalVector(t0);

					int numSegments = 8;
					double tubeRadius = g->bbox().diagonal().norm() * 0.03;

					double theta = (M_PI * 2.0) / numSegments;

					// First cross-section
					for(int i = 0; i < numSegments; i++){
						crossSection.push_back(start);
						start = rotatedVec(start, theta, t0);
					}

					// Tube
					for(int i = 0; i < N - 1; i++)
					{
						Vector3 ti = (skeleton[i+1] - skeleton[i]).normalized();

						// Rotate cross-section
						Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(t0, ti).normalized();

						for(int j = 0; j < crossSection.size(); j++)
							crossSection[j] = q * crossSection[j];

						crossSections.push_back(crossSection);

						// Set to new normal
						t0 = ti;
					}

					// Draw tube
					for(int i = 0; i < crossSections.size() - 1; i++)
					{
						glBegin(GL_QUAD_STRIP);
						for(int j = 0; j < numSegments + 1; j++)
						{
							int k = j % numSegments;
							Vector3 a = (crossSections[i][k] * tubeRadius) + skeleton[i];
							Vector3 b = (crossSections[i+1][k] * tubeRadius) + skeleton[i+1];

							glVector3(a);
							glVector3(b);
						}
						glEnd();
					}
				}
			}
		}

		glEnable(GL_LIGHTING);
	}

	// Markers used for matching
	marker.draw();

	// DEBUG:
	ps1.draw(); ps2.draw();	vs1.draw(3); vs2.draw();	ss1.draw(); ss2.draw();

    // Restore
    glViewport(viewport[0],viewport[1],viewport[2],viewport[3]);
}

void GraphItem::pick( int X, int Y, int pickMode )
{
	if(!m_geometry.contains(X,Y)){
		emit( miss() );
		return;
	}

	int x = X - m_geometry.x();
	int y = Y - m_geometry.y();

	qglviewer::Vec orig, dir;
	QMap< double, QPair<int, Vector3> > isects;
	QMap< double, QString > isectNode;

	this->setCamera();
	camera->convertClickToLine(QPoint(x,y), orig, dir);
	
	foreach(Structure::Node * n, g->nodes)
	{
		SurfaceMesh::Model* nodeMesh = g->getMesh(n->id);
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
				isects[dist] = qMakePair(f.idx(), ipoint);
				isectNode[dist] = n->id;
			}
		}
	}

	// Only consider closest hit
	if(isects.size())
	{
		double minDist = isects.keys().front();
		Vector3 ipoint = isects[minDist].second;
		Vector3 direction(-dir[0],-dir[1],-dir[2]);
		int faceIDX = isects[minDist].first;

		emit( hit( HitResult(this, isectNode[minDist], ipoint, QPoint(X,Y), faceIDX, pickMode) ) );

		// DEBUG:
		if(false){
			direction *= 0.1;
			ss1.addSphere(ipoint, 0.03f);
			vs1.addVector(ipoint, direction);
		}
	}
	else
	{
		emit( miss() );
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
