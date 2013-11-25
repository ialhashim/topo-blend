#include "RelationDetector.h"

QTextStream& operator << (QTextStream& os, const PairRelation& pr)
{    
	os << pr.type << " Pair <" << pr.n1->id << ", " << pr.n2->id << "> size <" << pr.n1->bbox().diagonal().norm() << ", " << pr.n2->bbox().diagonal().norm() << 
		"> deviation: " << pr.deviation << "\n";
    return os;
}
double RelationDetector::computePairDiameter(PairRelation& pr)
{
	Eigen::AlignedBox3d bbox1 = pr.n1->bbox();
	Eigen::AlignedBox3d bbox2 = pr.n2->bbox();
	Eigen::AlignedBox3d bbox = bbox2.merged(bbox1);
	return bbox.diagonal().norm();
}
double RelationDetector::fixDeviationByPartName(QString& s1, QString& s2, double deviation)
{
	double ndeviation = deviation;
    int idx = s1.indexOf(QRegExp("\\d"), 0);
    QString str1 = s1.left(idx);
    QString str2 = s2.left(idx);
    if ( str1 == str2)
        ndeviation *= 0.5;

	return ndeviation;
}
Eigen::MatrixXd RelationDetector::node2matrix(Structure::Node* node, int pointLevel)
{
	std::vector<Eigen::Vector3d> nodeCptsV;
	extractCpts( node, nodeCptsV, pointLevel);
	Eigen::MatrixXd nodeCptsM;
	vectorPts2MatrixPts(nodeCptsV, nodeCptsM);
	return nodeCptsM;
}
int RelationDetector::extractCpts( Structure::Node * n, std::vector<Eigen::Vector3d>& mcpts, int pointsLevel)
{
    if ( pointsLevel == 2)
    {
    	SurfaceMesh::Model * m1 = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data();
    	SurfaceMesh::Vector3VertexProperty pts1 = m1->vertex_coordinates();
    	double* tmp;
    	foreach(Vertex v1, m1->vertices())
    	{
    		tmp = pts1[v1].data();
    		mcpts.push_back( Eigen::Vector3d(tmp[0], tmp[1], tmp[2]) );
    	}
    }
	else
	{
		if ( Structure::CURVE == n->type() )
		{
			Structure::Curve* c = dynamic_cast<Structure::Curve *>(n);
			if ( pointsLevel == 1)
			{
				for (int i = 0; i < (int) c->numCtrlPnts(); ++i)
				{
					mcpts.push_back( c->controlPoint(i));
				}
			}
			else
			{
				mcpts.push_back( c->controlPoint(0) );
				mcpts.push_back( c->controlPoint( c->numCtrlPnts()-1 ) );
			}
		}
		else
		{
			Structure::Sheet* s = dynamic_cast<Structure::Sheet *>(n);
			if ( pointsLevel == 1)
			{
				for (int i = 0; i < (int) s->numCtrlPnts(); ++i)
				{
					mcpts.push_back( s->controlPoint(i));
				}
			}
			else
			{
				int nu = s->numUCtrlPnts(), nv = s->numVCtrlPnts();
				mcpts.push_back( s->surface.GetControlPoint(0,0) );
				mcpts.push_back( s->surface.GetControlPoint(nu-1,0) );
				mcpts.push_back( s->surface.GetControlPoint(nu-1,nv-1) );
				mcpts.push_back( s->surface.GetControlPoint(0,nv-1) );
			}
		}
	}

	return mcpts.size();
}
void RelationDetector::vectorPts2MatrixPts(const std::vector<Eigen::Vector3d>& ptsin, Eigen::MatrixXd& ptsout)
{
	ptsout.resize(ptsin.size(), 3);
    for ( int i = 0; i < (int)ptsin.size(); ++i)
	{
		ptsout.row(i) = ptsin[i];
	}
}
std::vector<Eigen::Vector3d> RelationDetector::matrixPts2VectorPts(Eigen::MatrixXd& ptsin)
{
	std::vector<Eigen::Vector3d> ptsout;
	for ( int i = 0; i < ptsin.rows(); ++i)
	{
		ptsout.push_back( ptsin.row(i));
	}
	return ptsout;
}

Eigen::Vector3d RelationDetector::curve2vectorNormalized(Structure::Node * n)
{
	const Vector3& bpn = n->controlPoint(0);
	const Vector3& epn = n->controlPoint(n->numCtrlPnts()-1);
	Eigen::Vector3d vec = bpn - epn;
	vec.normalize();
	return vec;
}
std::vector<Point_3> RelationDetector::sheet2rect(Structure::Node * n)
{
	Structure::Sheet* s = dynamic_cast<Structure::Sheet *>(n);
	std::vector<Point_3> result;
	Vector3 v = s->surface.GetControlPoint(0,0); 
	result.push_back( Point_3(v.x(), v.y(), v.z()) );
	v = s->surface.GetControlPoint(s->surface.GetNumCtrlPoints(0)-1,0); 
	result.push_back( Point_3(v.x(), v.y(), v.z()) );
	v = s->surface.GetControlPoint(0,s->surface.GetNumCtrlPoints(1)-1); 
	result.push_back( Point_3(v.x(), v.y(), v.z()) );
	v = s->surface.GetControlPoint(s->surface.GetNumCtrlPoints(0)-1,s->surface.GetNumCtrlPoints(1)-1); 
	result.push_back( Point_3(v.x(), v.y(), v.z()) );
	return result;
}
Segment_3 RelationDetector::curve2segment(Structure::Node * n)
{
	const Vector3& bpn = n->controlPoint(0);
	const Vector3& epn = n->controlPoint(n->numCtrlPnts()-1);

	Segment_3 seg(Point_3(bpn.x(),bpn.y(),bpn.z()), Point_3(epn.x(),epn.y(),epn.z()));
	return seg;
}
Line_3 RelationDetector::curve2line(Structure::Node * n)
{
	return Line_3(curve2segment(n));
}
void RelationDetector::sheet2plane(Structure::Sheet * s, Vector3& pos, Vector3& normal)
{
	std::vector<Vector3> nf = noFrame();
	s->get(Vector4d(0.5,0.5,0,0), pos, nf);
	normal = nf[2];
}
bool RelationDetector::node2direction(Structure::Node * n, Vector_3& result)
{
	bool iscurve(true);
	if ( 0 == Structure::CURVE.compare( n->type()) )
	{
		Line_3 line = curve2line(n);
		result = line.direction;		
	}
	else
	{
		Vector3 point, normal;
		sheet2plane(dynamic_cast<Structure::Sheet *>(n), point, normal);
		result = Vector_3(normal.x(), normal.y(), normal.z());
		iscurve = false;
	}	
	result = result/sqrt(result.squaredNorm());
	return iscurve;
}

// bSource == true means that the id is from target shape, & we want to find its corresponded node in source shape, then graph should be source
std::vector<Structure::Node*> RelationDetector::findNodesInST(QString id, Structure::Graph *graph, QVector<PART_LANDMARK> &corres, bool bSource)
{	
	QVector<QString> ids;
    if ( bSource)
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
            QVector<QString> ids2 = corres[i].second;
			for ( int j = 0; j < ids2.size(); ++j)
            {
				if ( ids2[j] == id)
				{
					 ids = corres[i].first;
					 break;
				}
            }
			if ( !ids.isEmpty() )
				break;
		}
	}
	else
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
            QVector<QString> ids1 = corres[i].first;
			for ( int j = 0; j < ids1.size(); ++j)
            {
				if ( ids1[j] == id)
				{
					 ids = corres[i].second;
					 break;
				}
            }
			if ( !ids.isEmpty() )
				break;
		}
	}

	std::vector<Structure::Node*> result;
	for ( int i = 0; i < (int) ids.size(); ++i)
	{
		result.push_back( graph->getNode(ids[i]) );
	}
	return result;
}

std::vector<Structure::Node*> RelationDetector::findNodesInB(QString id, Structure::Graph *graph, QVector<PART_LANDMARK> &corres, bool bSource)
{
    std::vector<Structure::Node*> result;

    if ( bSource)
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
            QVector<QString> ids = corres[i].first;
            int numCorres = corres[i].second.size();
            if ( 1 == numCorres && ids[0] == id)
            {
                Structure::Node* tmpNode = graph->getNode(id);
                if ( tmpNode != NULL)
                    result.push_back( tmpNode );

                return result;
            }
            else if ( 1 < numCorres )
            {
                if ( ids[0].startsWith( id + "_", Qt::CaseSensitive) )
                {
                    for ( int j = 0; j < numCorres; ++j)
                    {
						Structure::Node* tmpNode = graph->getNode(ids[j]);
						if ( tmpNode != NULL)
	                        result.push_back( tmpNode);
                    }
                    return result;
                }
            }
        }
		Structure::Node* tmpNode = graph->getNode(id);
		if ( tmpNode != NULL)
			result.push_back( graph->getNode(id) );   
    }
    else // is target
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
            QVector<QString> ids1 = corres[i].first;
            QString id2 = corres[i].second[0];
            if ( id2 == id)
            {
                for ( int j = 0; j < ids1.size(); ++j)
                {
					Structure::Node* tmpNode = graph->getNode(ids1[j]);
					if ( tmpNode != NULL)
						result.push_back( tmpNode );
                }
                return result;
            }
        }
        //
        Structure::Node* tmpNode = graph->getNode(id + "_null");
        if ( tmpNode != NULL)
        {
            result.push_back( tmpNode );            
        }		
    }
    //result.push_back( graph->getNode(id) );   
	return result;
}

void RelationDetector::createPlane(Structure::Node *n1,Structure::Node *n2, Eigen::Vector3d& point, Eigen::Vector3d& normal)
{
    Segment_3 s0 = curve2segment(n1);
    Segment_3 s2 = curve2segment(n2);

    // use middle point of 2 segment to generate a new segment, avoiding that s0 & s2 are parallel
    Point_3 p0( 0.5*(s0.point(0).x() + s0.point(1).x()),
            0.5*(s0.point(0).y() + s0.point(1).y()),
            0.5*(s0.point(0).z() + s0.point(1).z())  );
    Point_3 p2(0.5*(s2.point(0).x() + s2.point(1).x()),
            0.5*(s2.point(0).y() + s2.point(1).y()),
            0.5*(s2.point(0).z() + s2.point(1).z()) );
    Segment_3 s1 = Segment_3(p0,p2);

    ///////
    Point_3 cp( 0.25*(s0.point(0).x() + s0.point(1).x() + s2.point(0).x() + s2.point(1).x()),
        0.25*(s0.point(0).y() + s0.point(1).y() + s2.point(0).y() + s2.point(1).y()),
        0.25*(s0.point(0).z() + s0.point(1).z() + s2.point(0).z() + s2.point(1).z()) );
    point[0] = cp.x(); point[1] = cp.y(); point[2] = cp.z();

    Vector_3 v0 = cross_product(s1,s0);
    Vector_3 v2 = cross_product(s1,s2);
    if ( v0.squaredNorm() > v2.squaredNorm())
    {
        normal[0] = v0.x(); normal[1] = v0.y(); normal[2] = v0.z();
    }
    else
    {
        normal[0] = v2.x(); normal[1] = v2.y(); normal[2] = v2.z();
    }
    normal.normalize();
}