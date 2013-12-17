#include "RelationDetector.h"
#include <algorithm>
#include <numeric>
Q_DECLARE_METATYPE(Vector3)

bool isTaged(const PairRelation &pr)
{
	return pr.tag;
}
bool isTagedGr(const GroupRelation &gr)
{
	return gr.tag;
}

void saveToFile(QString filename, QVector<PairRelation>& prs)
{
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
		
	QTextStream out(&file);
	int i(0);
	out << "pair relations number: " << prs.size() << "\n";
	out << "    TRANS: " << countByType(prs,TRANS) << "\n";
	out << "    REF: " << countByType(prs,REF) << "\n\n";

	foreach(PairRelation pr, prs)
	{
		out << i << " " << pr.type << ": " << pr.n1->id << ", " << pr.n2->id << ", ";
		if ( pr.type == TRANS)
			out << "Trans: ["<<pr.trans_vec[0]<<", "<<pr.trans_vec[1]<<", "<<pr.trans_vec[2]<< "]\n";		
		else
			out << "\n";
		out << ", diameter: " << pr.diameter << "\n\n";
		++i;
	}
	file.close();
}
void saveToFile(QString filename, QVector<GroupRelation>& grs)
{
	QFile file(filename);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return;
		
	QTextStream out(&file);
	// output groups info
	int i(0);
	foreach(GroupRelation gr, grs)
	{
		out << i << " <group-" << gr.type << ">: ";
		if ( gr.type == REF_SYMMETRY)
		{
			out << "\n Point: " << gr.point.x() << "," << gr.point.y() << "," << gr.point.z() << "\n";
			out << "normal: " << gr.normal.x() << "," << gr.normal.y() << "," << gr.normal.z() << "\n";
		}
		else
			out << "\n";

		foreach(QString id, gr.ids){
			out << id << ", ";
		}
		out << "</group>\n";
		out << "diameter: " << gr.diameter << "\n";
		out << "deviation: " << gr.deviation << "\n\n";
		++i;
	}
	file.close();
}

QTextStream& operator << (QTextStream& os, const PairRelation& pr)
{    
	os << pr.type << " Pair <" << pr.n1->id << ", " << pr.n2->id << "> size <" << pr.n1->bbox().diagonal().norm() << ", " << pr.n2->bbox().diagonal().norm() << 
		"> deviation: " << pr.deviation << "\n";
    return os;
}
QTextStream& operator << (QTextStream& os, const GroupRelation& gr)
{    
    os << "Group <";
	for (int i = 0; i < gr.ids.size(); ++i)
	{
		os << gr.ids[i] << ", ";
	}
	os << gr.type << ">\n";
	
	if ( gr.type == AXIS_SYMMETRY)
	{
		os << "Center: " << gr.center.x() << ", " << gr.center.y() << ", " << gr.center.z() << "\n";
		os << "Direction: " << gr.direction.x() << ", " << gr.direction.y() << ", " << gr.direction.z() << "\n";
	}
	else if ( gr.type == REF_SYMMETRY)
	{
		os << "normal of plane: " << gr.normal.x() << ", " << gr.normal.y() << ", " << gr.normal.z() << "\n";
		os << "point of plane: " << gr.point.x() << ", " << gr.point.y() << ", " << gr.point.z() << "\n";
	}

	os << "Diameter: " << gr.diameter << "\n";
	os << "Deviation: " << gr.deviation << "\n\n";
    return os;
}
bool GroupRelation::equal(GroupRelation& gr)
{
    if (type.compare(gr.type))
        return false;

    if (ids.size() != gr.ids.size())
        return false;

    QVector<QString>::iterator it2 = gr.ids.begin();
    for ( QVector<QString>::iterator it1=ids.begin(); it1 != ids.end(); ++it1, ++it2)
    {
        if ( *it1 != *it2)
            return false;
    }
    return true;
}

Structure::Link* RelationDetector::findLink(Structure::Node *n1, Structure::Node * n2, Structure::Graph * graph)
{
	Structure::Link* link(0);
	int tmp1 = graph->edges.size();
	for ( int i = 0; i < tmp1; ++i)
	{
		Structure::Link* tmpLink = graph->edges[i];
		if (tmpLink->n1 == n1 && tmpLink->n2 == n2 || tmpLink->n1 == n2 && tmpLink->n2 == n1)
		{
			link = tmpLink;
			break;
		}		
	}
	return link;
}
double RelationDetector::computeDeviationByLink(Structure::Link* link)
{
	double deviation(0.0);
	if (link)
	{
		SurfaceMesh::Vector3 p1 = link->position(link->n1->id);
		SurfaceMesh::Vector3 p2 = link->position(link->n2->id);
		deviation = (p1-p2).norm();
		//deviation = link->property["blendedDelta"].value<Vector3>().norm();
	}
	return deviation;
}

RelationDetector::RelationDetector(Structure::Graph* g, const QString& logprefix, int ith, double normalizeCoef, int pointLevel, int logLevel)
	                              :graph_(g),logLevel_(logLevel), normalizeCoef_(normalizeCoef), pointLevel_(pointLevel)
{
	thRadiusRadio_ = 1.2;

    thTransRadio_ = 0.03; //1.3
    thRefRadio_ = 0.03;
    thAxisDeviationRadio_ = 0.9;
    thCopla_ = 0.002;//0.1
    thParal_ = 0.001;
    thOthog_ = 0.001;//0.1

    thAxisGroup_ = 0.01;
    thRefGroup_ = 0.01;
    thCoplaGroup_ = 0.003;


	if ( logLevel_ > 0)
	{			
		logFile_.setFileName(logprefix + QString::number(ith) + ".log");
		if (!logFile_.open(QIODevice::WriteOnly | QIODevice::Text)) return;		
		logStream_.setDevice(&logFile_);
	}
}

double RelationDetector::computePairDiameter(PairRelation& pr)
{
	Eigen::AlignedBox3d bbox1 = pr.n1->bbox();
	Eigen::AlignedBox3d bbox2 = pr.n2->bbox();
	Eigen::AlignedBox3d bbox = bbox2.merged(bbox1);
	return bbox.diagonal().norm();
}
double RelationDetector::computeGroupDiameter(GroupRelation& gr)
{
    Eigen::AlignedBox3d bbox;
    for ( QVector<QString>::iterator it = gr.ids.begin(); it!=gr.ids.end(); ++it)
    {
        Eigen::AlignedBox3d bbox1 = graph_->getNode(*it)->bbox();
        bbox = bbox.merged(bbox1);
    }
    //return bbox.volume();
    return bbox.diagonal().norm();
}
void RelationDetector::computeGroupCenter(GroupRelation& gr)
{
    Eigen::Vector3d center(0,0,0);
    for ( QVector<QString>::iterator it = gr.ids.begin(); it != gr.ids.end(); ++it)
    {
        Structure::Node* n1 = graph_->getNode(*it);
		center += computeCptsCenter(n1);
        //center += n1->center();
    }
    center = center / gr.ids.size();
    gr.center = Point_3(center.x(), center.y(), center.z());
}
double RelationDetector::fixDeviationByPartName(const QString& s1, const QString& s2, double deviation, double times)
{
	double ndeviation = deviation;
    int idx = s1.indexOf(QRegExp("\\d"), 0);
    QString str1 = s1.left(idx);
    QString str2 = s2.left(idx);
    if ( str1 == str2)
        ndeviation *= times;

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
            QVector<QString> ids1 = corres[i].first;
			for ( int j = 0; j < ids1.size(); ++j)
			{
				if ( id == ids1[j])
				{
					QVector<QString> ids2 = corres[i].second;
					if ( ids1.size() >= ids2.size()) // 1-1 or *-1
					{
						Structure::Node* tmpNode = graph->getNode(id);
						if ( tmpNode != NULL)
							result.push_back( tmpNode );
					}
					else // 1-*
					{
						for ( int k = 0; k < ids2.size(); ++k)
						{
							QString str; str.setNum(k);  
							Structure::Node* tmpNode = graph->getNode(id + "_" + str );
							if ( tmpNode != NULL)
								result.push_back( tmpNode);
						}						
					}
					return result;
				}
			}
        }
		// 1-0
        Structure::Node* tmpNode = graph->getNode(id);
        if ( tmpNode != NULL)
            result.push_back( tmpNode );            
    }
    else // is target
    {
        for ( int i = 0; i < (int) corres.size(); ++i)
        {
			QVector<QString> ids2 = corres[i].second;
			for ( int j = 0; j < ids2.size(); ++j)
			{
				if ( id == ids2[j])
				{
					 QVector<QString> ids1 = corres[i].first;
					 if ( ids1.size() >= ids2.size()) // 1-1 or *-1
					 {
						 for ( int k = 0; k < ids1.size(); ++k)
						 {
							Structure::Node* tmpNode = graph->getNode(ids1[k]);
							if ( tmpNode != NULL)
								result.push_back( tmpNode );
						 }
					 }
					 else // 1-*
					 {
						QString str; str.setNum(j);  
						Structure::Node* tmpNode = graph->getNode(ids1[0] + "_" + str );
						if ( tmpNode != NULL)
							result.push_back( tmpNode);
					 }
					 return result;
				}
			}
        }
        // 0-1
        Structure::Node* tmpNode = graph->getNode(id + "_null");
        if ( tmpNode != NULL)
            result.push_back( tmpNode );            
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

double RelationDetector::computeRefSymmetryGroupDeviation(GroupRelation& gr, int pointLevel)
{
    std::vector<Structure::Node*> nodes;
    for ( int i = 0; i < (int) gr.ids.size(); ++i)
    {
        nodes.push_back( graph_->getNode(gr.ids[i]) );
    }

    double minErr = std::numeric_limits<double>::max();
    std::pair<int, int> minNodes;
    Eigen::Vector3d point, normal;
    double err;
    for ( int i = 0; i < (int) gr.ids.size(); ++i)
    {
        for ( int j = i+1; j < (int) gr.ids.size(); ++j)
        {
            findRefPlane(nodes[i], nodes[j], point,normal);
			if ( normal.squaredNorm() == 0)
				continue;

			err = errorOfRefSymmGroup(nodes, point, normal, pointLevel);

            if ( err < minErr)
            {
                minErr = err;
                minNodes = std::make_pair(i,j);
            }
        }
    }

    findRefPlane(nodes[minNodes.first], nodes[minNodes.second], gr.point, gr.normal);
	if ( gr.normal.squaredNorm() == 0)
		minErr = 0;
	else
		minErr = errorOfRefSymmGroup(nodes, gr.point, gr.normal, pointLevel);

    return minErr/gr.ids.size();
}

double RelationDetector::computeAxisSymmetryGroupDeviation(GroupRelation& gr, int pointLevel)
{
    double result(0.0);
    int n = gr.ids.size();
    double angle = 2*3.1415926/n;

    QVector<QString> ids;
    ids.push_back(gr.ids.last());
    gr.ids.pop_back();

	double min_dist, mean_dist, max_dist;
    while(gr.ids.size())
    {
        Structure::Node* n1 = graph_->getNode(ids.last());
		Eigen::MatrixXd verts1 = node2matrix(n1, pointLevel);

        double err(std::numeric_limits<double>::max());
        int k(0);
        for ( int i = 0; i < (int) gr.ids.size(); ++i)
        {
            Structure::Node* n2 = graph_->getNode(gr.ids[i]);
			Eigen::MatrixXd verts2 = node2matrix(n2, pointLevel);
			Eigen::MatrixXd newverts2;            
			rotate_points3d(verts2, gr.center, gr.direction, angle, newverts2);
			distanceBetween(verts1, newverts2, min_dist, mean_dist, max_dist);
            if ( mean_dist < err)
			{
                err = mean_dist;
                k = i;
            }
        }
        result += err;
        ids.push_back(gr.ids[k]);
        gr.ids.remove(k);
    }
    for ( QVector<QString>::iterator it = ids.begin(); it != ids.end(); ++it)
        gr.ids.push_back(*it);

    return result/(ids.size()-1);
}

void RelationDetector::findRefPlane(Structure::Node* n1, Structure::Node* n2, Eigen::Vector3d& center,Eigen::Vector3d& normal)
{
    Eigen::Vector3d c1 = computeCptsCenter(n1);
    Eigen::Vector3d c2 = computeCptsCenter(n2);
    normal = c1 - c2;
	if ( normal.squaredNorm() != 0)
		normal.normalize();
    center = (c1+c2)*0.5;
}
Eigen::Vector3d RelationDetector::computeCptsCenter(Structure::Node* nn)
{
    Eigen::MatrixXd verts;
    vectorPts2MatrixPts(nn->controlPoints(), verts);
    Eigen::Vector3d center( verts.col(0).mean(), verts.col(1).mean(),verts.col(2).mean() );
    return center;
}
double RelationDetector::errorOfRefSymmGroup(std::vector<Structure::Node*> &nodes, Eigen::Vector3d& center, Eigen::Vector3d& normal, int pointLevel)
{
    std::vector<double> error;
    std::vector<bool> bDone(nodes.size(),false);
	double min_dist, mean_dist, max_dist;

    for ( int i = 0; i < (int) nodes.size(); ++i)
    {
        if ( bDone[i] ) continue;

		Eigen::MatrixXd newverts1;
        reflect_points3d(node2matrix(nodes[i], pointLevel), center, normal, newverts1);
        double err = std::numeric_limits<double>::max();
        int minIdx(0);

        for ( int j = i; j < (int) nodes.size(); ++j)
        {				
            distanceBetween( node2matrix(nodes[j], pointLevel), newverts1, min_dist, mean_dist, max_dist);
            if ( mean_dist < err)
            {
                err = mean_dist;
                minIdx = j;
            }
        }
        error.push_back(err);
        bDone[i] = true; bDone[minIdx] = true;
    }
    return std::accumulate( error.begin(), error.end(), 0.0);
}

void RelationDetector::computeTransGroupInfo(GroupRelation &gr, QSet<QString>& ids)
{
    QSet<QString>::iterator it = ids.begin();
    Structure::Node* n = graph_->getNode(*it);    
    Vector_3 direction(0,0,0);
    node2direction(n, direction);
    gr.ids.push_back(*it);
    ++it;
    Vector_3 dc;
    int k(1);
    for (; it!=ids.end(); ++it,++k )
    {
        Structure::Node* nn = graph_->getNode(*it);        

        node2direction(nn, dc);
        if ( dot(direction, dc) > 0)
            direction = direction + dc;
        else
            direction = direction - dc;

        direction = direction/sqrt(direction.squaredNorm());
        gr.ids.push_back(*it);
    }
    
    gr.direction = direction;
    gr.diameter = computeGroupDiameter(gr);
	computeGroupCenter(gr);
}