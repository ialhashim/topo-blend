#pragma once

#include <vector>
#include <algorithm>
#include <sstream>
#include <numeric>

#include "StructureGraph.h"
#include "StructureNode.h"
#include "GraphCorresponder.h"

#include "transform3d.h"

#include "Geometry.h"

static QString TRANS = "TRANS";
static QString PARALLEL = "PARALLEL";
static QString REF = "REF";
static QString ORTHOGONAL = "ORTHOGONAL";
static QString COPLANAR = "COPLANAR";
static QString NONE = "NONE";

static QString TRANS_SYMMETRY = "TRANS_SYMMETRY"; // build from Trans pair, except AXIS_SYMMETRY
static QString AXIS_SYMMETRY = "AXIS_SYMMETRY";
static QString REF_SYMMETRY = "REF_SYMMETRY";

struct PairRelation
{
    PairRelation():tag(false){}
    PairRelation(QString& id1_, QString& id2_, QString& type_):id1(id1_),id2(id2_),type(type_),tag(false){}
    bool equal(PairRelation& pr)
    {
        if (type.compare(pr.type))
            return false;
        QString tmp1(id1+id2);
        QString tmp2(id2+id1);
        if (tmp1.compare(pr.id1+pr.id2) && tmp2.compare(pr.id1+pr.id2))
            return false;
        return true;
    }
    QString id1;
    QString id2;
    QString type;
    bool tag;
    double deviation;
    double diameter;

    Eigen::Vector3d trans_vec;// for TRANS

    //// for co-axis symmetry
    //Eigen::Vector3d center;
    //Eigen::Vector3d direction;

    Eigen::Vector3d normal;
    Eigen::Vector3d point;

    friend QTextStream& operator<<(QTextStream& os, const PairRelation& pr);
};

struct GroupRelation
{
	GroupRelation(){}

    QVector<QString> ids; //QSet<QString> ids;
    QString type;
    double diameter; // group diameter
    double deviation; // absolute mean deviation
    bool tag;

    Point_3 center; //
    Vector_3 direction; // normalized. line direction for AXIS_SYMMETRY, normal direction for REF_SYMMETRY
    Line_3 axis; // for co-axis symmetry or co-axis parallel symmetry
    Plane_3 refPlane; // for ref symm group
    // for coplanar group
    Eigen::Vector3d normal;
    Eigen::Vector3d point;

    bool equal(GroupRelation& gr)
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

    friend QTextStream& operator<<(QTextStream& os, const GroupRelation& gr);
};

void vectorId2SetId(QVector<QString>& vec, QSet<QString>& set);
void setId2VectorId(QSet<QString>& set, QVector<QString>& vec);
void mini(Vector3& vin, Vector3& vout);
void maxm(Vector3& vin, Vector3& vout);
Segment_3 curve2segment(Structure::Node * n);
Line_3 curve2line(Structure::Node * n);
Eigen::Vector3d curve2vectorNormalized(Structure::Node * n);

std::vector<Point_3> sheet2rect(Structure::Node * n);
void sheet2plane(Structure::Sheet * s, Vector3& point, Vector3& normal);
bool node2direction(Structure::Node * n, Vector_3& result);
bool typeLessThan(const PairRelation &pr1, const PairRelation &pr2);
bool isTaged(const PairRelation &pr);
bool isTagedGr(const GroupRelation &gr);
template<class T>
int countByType(QVector<T>& prs, QString& type)
{
    int i(0);
    foreach(T pr, prs)
    {
        if ( 0 == type.compare(pr.type))
            ++i;
    }
    return i;
}

void saveToFile(QString filename, QVector<PairRelation>& prs);
void saveToFile(QString filename, QVector<GroupRelation>& grs);
bool isInGroupById(QString id, QVector<QString>& group);
QString group2str(QVector<QString> &group);
bool isInGroups(QVector<QString> &group, NodeGroups &planarGroups);
double errorOfLineInPlane(Segment_3 &l, Vector3& point, Vector3& normal);
static void saveScores(QVector<double>& scoreKeptGr, //QVector<double>& scoreAddGr,
	QVector<double>& scoreKeptPr,//QVector<double>& scoreAddPr,
	QTextStream& out);

class RelationDetector
{
public:
    double thRadiusRadio_;
    double thTransRadio_;
    double thRefRadio_;
    double thParal_;
    double thOthog_;
    double thCopla_;

    double thRefGroup_;
    double thAxisGroup_;
    double thAxisDeviationRadio_;
    double thCoplaGroup_;

    double thDegeneratedNode_;

    Structure::Graph* graph_;
    RelationDetector(Structure::Graph* g):graph_(g)
    {
        thRadiusRadio_ = 1.2;

        thTransRadio_ = 0.03; //1.3
        thRefRadio_ = 0.06;
        thAxisDeviationRadio_ = 0.9;
        thCopla_ = 0.002;//0.1
        thParal_ = 0.001;
        thOthog_ = 0.001;//0.1

        thAxisGroup_ = 0.01;
        thRefGroup_ = 0.01;
        thCoplaGroup_ = 0.003;

        thDegeneratedNode_ = 0.001;
    }
    ~RelationDetector()
    {
    }
protected:

    double errorOfParallel(Eigen::Vector3d& d1, Eigen::Vector3d& d2)
    {
        return 1 - sqrt( std::abs(d1.dot(d2))/(d1.norm()*d2.norm()) );
    }
    double errorOfOrthogonal(Eigen::Vector3d& d1, Eigen::Vector3d& d2)
    {
        return sqrt( std::abs(d1.dot(d2))/(d1.norm()*d2.norm()) );
    }
    double errorOfCoplanar(Vector3 &pt1, Vector3 &normal1, Vector3 &pt2, Vector3 &normal2)
    {
        double err1 = 1-std::abs(normal1.dot(normal2));

        Point_3 point(pt1.x(), pt1.y(), pt1.z());
        Plane_3 p2(Point_3(pt2.x(), pt2.y(), pt2.z()), Vector_3(normal2.x(), normal2.y(), normal2.z()));
        double dist1 = squared_distance(p2, point);
        return err1 + sqrt(dist1);
    }
    //
    Eigen::Vector3d computeCptsCenter(Structure::Node* nn)
    {
        Eigen::MatrixXd verts;
        vectorPts2MatrixPts(nn->controlPoints(), verts);
        Eigen::Vector3d center( verts.col(0).mean(), verts.col(1).mean(),verts.col(2).mean() );
        return center;
    }

    void meshVerts2MatrixPts(Structure::Node* n1, Eigen::MatrixXd& ptsout) //std::vector<Eigen::Vector3d>& ptsin)
    {
        //if(n1->property.contains("mesh"))
        SurfaceMesh::Model * m1 = n1->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data();
        SurfaceMesh::Vector3VertexProperty pts1 = m1->vertex_coordinates();
        int i(0);double* tmp;
        ptsout.resize(m1->vertices_size(),3);
        foreach(Vertex v1, m1->vertices())
        {
            tmp = pts1[v1].data();
            ptsout.row(i) = Eigen::Vector3d(tmp[0], tmp[1], tmp[2]);
            ++i;
        }
    }
    void transNodeMeshVerts(Structure::Node*n, Eigen::Vector3d& center, Eigen::Vector3d& normal, Eigen::MatrixXd & newverts)
    {
        meshVerts2MatrixPts(n, newverts);
        for ( int i = 0; i < (int) newverts.rows(); ++i)
        {
            Vector3 p1 = newverts.row(i);
            Vector3 v = normal * (center - p1).dot(normal);
            newverts.row(i) = p1 + v * 2;
        }
    }
    void transNodeMeshVerts(Structure::Node* n, Eigen::Vector3d transVec, Eigen::MatrixXd & newverts)
    {
        meshVerts2MatrixPts(n, newverts);
        Eigen::Matrix4d transMat = create_translation3d(transVec);
        newverts = transform_point3d(newverts, transMat);
    }
    double distanceBetween(const Eigen::MatrixXd& v1, const Eigen::MatrixXd& v2)
    {
        double err1(0.0);
        for(int i = 0; i < (int) v1.rows(); ++i)
        {
            double minErr = std::numeric_limits<double>::max(), val(0.0);
            Eigen::Vector3d pt1 = v1.row(i);
            for ( int j = 0; j < (int) v2.rows(); ++j)
            {
                Eigen::Vector3d pt2 = v2.row(j);
                val = (pt1-pt2).norm();
                if ( val < minErr)
                    minErr = val;
            }
            err1 += minErr;
        }
        err1 = err1/v1.rows();
        return err1;
    }
    void distanceBetween(const Eigen::MatrixXd& v1, const Eigen::MatrixXd& v2, double &mean_dist, double &max_dist)
    {
        mean_dist = 0.0; max_dist = 0.0;
        for(int i = 0; i < (int) v1.rows(); ++i)
        {
            double minErr = std::numeric_limits<double>::max(), val(0.0);
            Eigen::Vector3d pt1 = v1.row(i);
            for ( int j = 0; j < (int) v2.rows(); ++j)
            {
                Eigen::Vector3d pt2 = v2.row(j);
                val = (pt1-pt2).norm();
                if ( val < minErr)
                    minErr = val;
            }
            mean_dist += minErr;
            if(minErr > max_dist)
                max_dist = minErr;
        }
        mean_dist = mean_dist/v1.rows();
    }
    double distanceBetweenTransNodesMesh(Structure::Node* n1, Eigen::MatrixXd & verts2)
    {
        Eigen::MatrixXd verts1;
        meshVerts2MatrixPts(n1, verts1);
        double err1 = distanceBetween(verts1, verts2);
        double err2 = distanceBetween(verts2, verts1);
        return std::max(err1,err2);
    }
    // compute the center and direction of the group
    void computeTransGroupInfo(GroupRelation &gr, QSet<QString>& ids)
    {
        QSet<QString>::iterator it = ids.begin();
        Structure::Node* n = graph_->getNode(*it);
        Vector3 center = computeCptsCenter(n);
        Vector_3 direction(0,0,0);
        node2direction(n, direction);
        gr.ids.push_back(*it);
        ++it;
        Vector_3 dc;
        int k(1);
        for (; it!=ids.end(); ++it,++k )
        {
            Structure::Node* nn = graph_->getNode(*it);
            center += computeCptsCenter(nn);

            node2direction(nn, dc);
            if ( dot(direction, dc) > 0)
                direction = direction + dc;
            else
                direction = direction - dc;

            direction = direction/sqrt(direction.squaredNorm());
            gr.ids.push_back(*it);
        }
        center /= k;
        gr.center = Point_3(center.x(), center.y(), center.z());
        gr.direction = direction;
        gr.diameter = computeGroupDiameter(gr);
    }

    // relative to the diameter of the graph, i.e. model
    double computeGroupDiameter(GroupRelation& gr)
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
    double computePairDiameter(PairRelation& pr)
    {
        Eigen::AlignedBox3d bbox1 = graph_->getNode(pr.id1)->bbox();
        Eigen::AlignedBox3d bbox2 = graph_->getNode(pr.id2)->bbox();
        Eigen::AlignedBox3d bbox = bbox2.merged(bbox1);
        return bbox.diagonal().norm();
    }
    void findRefPlane(Structure::Node* n1, Structure::Node* n2, Eigen::Vector3d& center,Eigen::Vector3d& normal)
    {
        Eigen::Vector3d c1 = computeCptsCenter(n1);
        Eigen::Vector3d c2 = computeCptsCenter(n2);
        normal = c1 - c2;
        normal.normalize();
        center = (c1+c2)*0.5;
    }
    double computeScore(GroupRelation& cgr, GroupRelation& gr)
    {
        return gr.diameter*(gr.deviation+1)/(cgr.deviation+1);

        //return (gr.deviation+1)/(cgr.deviation+1);
        //return gr.diameter*log(gr.deviation+2)/(log(cgr.deviation+2));
    }
    double computeScore(PairRelation& cpr, PairRelation& pr)
    {
        return pr.diameter*(pr.deviation+1)/(cpr.deviation+1);

        //return (pr.deviation+1)/(cpr.deviation+1);
        //return pr.diameter*log(pr.deviation+2)/(log(cpr.deviation+2));
    }
    double errorOfCoplanarGroupByCpts(std::vector<Structure::Node*> &nodes, Eigen::Vector3d& point, Eigen::Vector3d& normal)
    {
        double err(0.0);
        Point_3 p(point[0],point[1],point[2]);
        Vector_3 n(normal[0],normal[1],normal[2]);
        Plane_3 plane(p, n);

        for ( int i = 0; i < (int) nodes.size(); ++i)
        {
            Eigen::Vector3d pt1 = nodes[i]->controlPoint(0);
            int ncp = nodes[i]->numCtrlPnts();
            Eigen::Vector3d pt2 = nodes[i]->controlPoint( ncp-1 );
            Point_3 p1(pt1.x(), pt1.y(), pt1.z());
            Point_3 p2(pt2.x(), pt2.y(), pt2.z());

            double dist1 = squared_distance(plane, p1);
            double dist2 = squared_distance(plane, p2);
            err += (dist1+dist2);
        }
        return err;
    }

    void computeGroupDeviationByCpts(GroupRelation& cgr, GroupRelation& gr)
    {
        cgr.type = gr.type;
        if ( cgr.type == AXIS_SYMMETRY)
        {
            cgr.deviation = computeAxisSymmetryGroupDeviation(cgr,false);
        }
        else if ( cgr.type == REF_SYMMETRY)
        {
            cgr.deviation = computeRefSymmetryGroupDeviation(cgr,false);
        }
        else if ( cgr.type == COPLANAR)
        {
            std::vector<Structure::Node*> cgrNodes;
            Structure::Node* n = graph_->getNode( cgr.ids[0]);
            cgrNodes.push_back(n);
            Eigen::Vector3d d0 = curve2vectorNormalized(n);
            Line_3 l0 = curve2line(n);
            std::vector<Eigen::Vector3d> pts, normals;

            for ( int j = 1; j < (int) cgr.ids.size(); ++j)
            {
                n = graph_->getNode( cgr.ids[j]);
                Eigen::Vector3d d1 = curve2vectorNormalized(n);
                double error = errorOfParallel(d0, d1);
                if (error < thParal_)
                {
                    Line_3 l1 = curve2line(n);
                    error = squared_distance(l0, l1);
                    if ( error < thDegeneratedNode_) // they are the same line
                        continue;
                }
                createPlane( cgrNodes[0], n, cgr.point, cgr.normal);
                pts.push_back(cgr.point);
                normals.push_back(cgr.normal);

                cgrNodes.push_back( n );
            }
            if ( pts.empty() )
                cgr.deviation = 0;
            else
            {
                cgr.point = Eigen::Vector3d::Zero();
                cgr.normal = Eigen::Vector3d::Ones();
                for ( int i = 1; i < (int) pts.size(); ++i)
                {
                    Eigen::Vector3d& nl = normals[i];
                    cgr.point = cgr.point + pts[i];
                    if ( cgr.normal.dot(nl) > 0)
                        cgr.normal = cgr.normal + nl;
                    else
                        cgr.normal = cgr.normal - nl;
                }
                cgr.point = cgr.point/pts.size();
                //cgr.normal=gr.normal/pts.size();
                cgr.normal.normalize();
                cgr.deviation = errorOfCoplanarGroupByCpts(cgrNodes, cgr.point, cgr.normal);
            }
            //if ( cgr.ids[0] == "BarLeft")
            //{
            //	logStream_ << "\n\n\n";
            //
            //	Vector_3 normal = cgr.plane.orthogonal_vector();
            //	logStream_ << "normal of plane: " << normal.x() << ", " << normal.y() << ", " << normal.z() << "\n";
            //}
        }

    }

	void rotNodeMeshVerts(Structure::Node* no, Point_3& center, Vector_3& direction, double angle, Eigen::MatrixXd& newverts)
	{
        Eigen::Vector3d c(center.x(), center.y(), center.z());
        Eigen::Vector3d dir(direction.x(), direction.y(), direction.z());
        Eigen::Matrix4d rotMat = create_rotation3d_line_angle(center, direction, angle);

        meshVerts2MatrixPts(no, newverts);
        newverts = transform_point3d(newverts, rotMat);
    }

	void rotNodeCpts(Structure::Node* no, Point_3& center, Vector_3& direction, double angle, Eigen::MatrixXd& newverts)
	{
        Eigen::Vector3d c(center.x(), center.y(), center.z());
        Eigen::Vector3d dir(direction.x(), direction.y(), direction.z());
        Eigen::Matrix4d rotMat = create_rotation3d_line_angle(center, direction, angle);

        vectorPts2MatrixPts(no->controlPoints(), newverts);
        newverts = transform_point3d(newverts, rotMat);
    }
    // translate n2 to n1, output n2
    Eigen::Vector3d transNodeCpts(Structure::Node *const n1, Structure::Node *const n2, Eigen::MatrixXd& ptsout)
    {
        Eigen::Vector3d trans = n1->center() - n2->center();

        vectorPts2MatrixPts(n2->controlPoints(), ptsout);
        Eigen::Matrix4d transMat = create_translation3d(trans);
        ptsout = transform_point3d(ptsout, transMat);
        return trans;
    }
    // reflect node n accroding to the plane specified by center & normal
    void reflectNodeCpts(Structure::Node *const n, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::MatrixXd& ptsout)
    {
        Eigen::MatrixXd ptsin;
        vectorPts2MatrixPts(n->controlPoints(), ptsin);
        reflectPoints(ptsin, center, normal, ptsout);
    }
    void reflectPoints(const Eigen::MatrixXd& ptsin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::MatrixXd& ptsout)
    {
        ptsout.resize(ptsin.rows(), ptsin.cols() );
        for ( int i = 0; i < (int) ptsin.rows(); ++i)
        {
            Eigen::Vector3d ptout;
            reflectPoint(ptsin.row(i), center, normal, ptout);
            ptsout.row(i) = ptout;
        }
    }
    void reflectPoint(const Eigen::Vector3d& ptin, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, Eigen::Vector3d& ptout)
    {
        Vector3 v = normal * (center - ptin).dot(normal);
        ptout = ptin + v * 2;
    }
    double distanceBetweenTransNodesCpts(Structure::Node* n1, Eigen::MatrixXd & cpts2)
    {
        Eigen::MatrixXd cpts1;
        vectorPts2MatrixPts(n1->controlPoints(), cpts1);
        double err1 = distanceBetween(cpts1, cpts2);
        double err2 = distanceBetween(cpts2, cpts1);
        return std::max(err1,err2);
    }
    double distanceBetweenTransNodesCpts(Structure::Node* n1, Structure::Node* n2)
    {
        Eigen::MatrixXd cpts2;
        vectorPts2MatrixPts(n2->controlPoints(), cpts2);
        return distanceBetweenTransNodesCpts(n1, cpts2);
    }
    // we do not order part in a group by ref plane
    // return mean deviation of the group
    double computeRefSymmetryGroupDeviation(GroupRelation& gr, bool bUseMeshVerts)
    {
        std::vector<Structure::Node*> nodes;
        for ( int i = 0; i < (int) gr.ids.size(); ++i)
        {
            nodes.push_back( graph_->getNode(gr.ids[i]) );
        }

        double minErr = std::numeric_limits<double>::max();
        std::pair<int, int> minNodes;
        Eigen::Vector3d center, normal;
        double err;
        for ( int i = 0; i < (int) gr.ids.size(); ++i)
        {
            for ( int j = i+1; j < (int) gr.ids.size(); ++j)
            {
                findRefPlane(nodes[i], nodes[j], center,normal);

                if (bUseMeshVerts)
                    err = errorOfRefSymmByMeshVerts(nodes, center, normal);
                else
                    err = errorOfRefSymmByCpts(nodes, center, normal);
                if ( err < minErr)
                {
                    minErr = err;
                    minNodes = std::make_pair(i,j);
                }
            }
        }

        findRefPlane(nodes[minNodes.first], nodes[minNodes.second], center,normal);
        gr.refPlane = Plane_3(Point_3(gr.center.x(), gr.center.y(), gr.center.z()), Vector_3(normal.x(), normal.y(), normal.z()) );
        return minErr/gr.ids.size()*2;
    }
        // order part in a group by angle around the axis
    // return mean deviation of the group
    double computeAxisSymmetryGroupDeviation(GroupRelation& gr, bool bUseMeshVerts)
    {
        double result(0);
        int n = gr.ids.size();
        double angle = 2*3.1415926/n;

        QVector<QString> ids;
        ids.push_back(gr.ids.last());
        gr.ids.pop_back();
        while(gr.ids.size())
        {
            Structure::Node* n1 = graph_->getNode(ids.last());
            double tmp(std::numeric_limits<double>::max());
            int k(0);
            for ( int i = 0; i < (int) gr.ids.size(); ++i)
            {
                Structure::Node* n2 = graph_->getNode(gr.ids[i]);
                Eigen::MatrixXd newverts2;
                double tmpSum;
                if ( bUseMeshVerts)
                {
                    rotNodeMeshVerts(n2, gr.center, gr.direction, angle, newverts2);
                    tmpSum = distanceBetweenTransNodesMesh(n1, newverts2);
                }
                else
                {
                    rotNodeCpts(n2, gr.center, gr.direction, angle, newverts2);
                    tmpSum = distanceBetweenTransNodesCpts(n1, newverts2);
                }

                if ( tmp > tmpSum)
                {
                    tmp = tmpSum;
                    k = i;
                }
            }
            result += tmp;
            ids.push_back(gr.ids[k]);
            gr.ids.remove(k);
        }
        for ( QVector<QString>::iterator it = ids.begin(); it != ids.end(); ++it)
            gr.ids.push_back(*it);

        return result/(ids.size()-1);
    }
// bSource == true means that the id is from source relation groups
    std::vector<Structure::Node*> findNodes(QString id, Structure::Graph *graph, QVector<PART_LANDMARK> &corres, bool bSource)
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
                    if ( tmpNode ==NULL)
                    {
                        for ( int j = 0; j < (int) numCorres; ++j)
                        {
                            result.push_back( graph->getNode(corres[i].second[j]) );
                        }
                    }
                    else
                    {
                        result.push_back( graph->getNode(id) );
                    }
                    return result;
                }
                else if ( 1 < numCorres )
                {
                    if ( ids[0].startsWith( id + "_", Qt::CaseSensitive) )
                    {
                        Structure::Node* tmpNode = graph->getNode(ids[0]);
                        if ( tmpNode ==NULL)
                        {
                            result.push_back( graph->getNode(corres[i].second[0]) );
                        }
                        else
                        {
                            for ( int j = 0; j < (int) ids.size(); ++j)
                            {
                                result.push_back( graph->getNode(ids[j]));
                            }
                        }
                        return result;
                    }
                }
            }
        }
        else // is target
        {
            for ( int i = 0; i < (int) corres.size(); ++i)
            {
                QVector<QString> ids1 = corres[i].first;
                QString id2 = corres[i].second[0];
                if ( id2 == id)
                {
                    Structure::Node* tmpNode = graph->getNode(ids1[0]);
                    if ( tmpNode ==NULL)
                    {
                        result.push_back( graph->getNode(id2) );
                    }
                    else
                    {
                        for ( int j = 0; j < (int) ids1.size(); ++j)
                        {
                            result.push_back( graph->getNode(ids1[j]) );
                        }
                    }
                    return result;
                }
            }
            //
            Structure::Node* tmpNode = graph->getNode(id + "_null");
            if ( tmpNode != NULL)
            {
                result.push_back( tmpNode );
                return result;
            }
        }

        result.push_back( graph->getNode(id) );
        return result;
    }
    void setInfoForDegeneratedGroup(GroupRelation &gr)
    {
        gr.deviation = std::numeric_limits<double>::max();
        gr.diameter = 0;
    }
    bool isDegenerateGroup(GroupRelation & gr)
    {
        return (0 == gr.diameter);
    }
    bool isDegeneratePair(PairRelation & pr)
    {
        return (0 == pr.diameter);
    }
    void setInfoForDegeneratedPair(PairRelation &pr)
    {
        pr.deviation = std::numeric_limits<double>::max();
        pr.diameter = 0;
    }
    // output:
    //        point
    //        normal
    // only work for two curve node
    void createPlane(Structure::Node *n1,Structure::Node *n2, Eigen::Vector3d& point, Eigen::Vector3d& normal)
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
private:
    double errorOfRefSymmByCpts(std::vector<Structure::Node*> &nodes, Eigen::Vector3d& center, Eigen::Vector3d& normal)
    {
        std::vector<double> error;
        std::vector<bool> bDone(nodes.size(),false);
        for ( int i = 0; i < (int) nodes.size(); ++i)
        {
            if ( bDone[i] ) continue;

            Structure::Node* n1 = nodes[i];
            Eigen::MatrixXd newverts1;
            reflectNodeCpts(n1, center, normal, newverts1);
            double err = std::numeric_limits<double>::max(), tmp;
            int minIdx(0);

            for ( int j = i; j < (int) nodes.size(); ++j)
            {
                Structure::Node* n2 = nodes[j];
                tmp = distanceBetweenTransNodesCpts(n2, newverts1);
                if ( tmp < err)
                {
                    err = tmp;
                    minIdx = j;
                }
            }
            error.push_back(err);
            bDone[i] = true; bDone[minIdx] = true;
        }
        return std::accumulate( error.begin(), error.end(), 0.0);
    }
    double errorOfRefSymmByMeshVerts(std::vector<Structure::Node*> &nodes, Eigen::Vector3d& center, Eigen::Vector3d& normal)
    {
        std::vector<double> error;
        std::vector<bool> bDone(nodes.size(),false);
        for ( int i = 0; i < (int) nodes.size(); ++i)
        {
            if ( bDone[i] ) continue;

            Structure::Node* n1 = nodes[i];
            Eigen::MatrixXd newverts1;
            transNodeMeshVerts(n1, center, normal, newverts1);
            double err = std::numeric_limits<double>::max(), tmp;
            int minIdx(0);
            for ( int j = i; j < (int) nodes.size(); ++j)
            {
                Structure::Node* n2 = nodes[j];
                tmp = distanceBetweenTransNodesMesh(n2, newverts1);
                if ( tmp < err)
                {
                    err = tmp;
                    minIdx = j;
                }
            }
            error.push_back(err);
            bDone[i] = true; bDone[minIdx] = true;
        }
        return std::accumulate( error.begin(), error.end(), 0.0);
    }
};
class GlobalReflectionSymmDetector : public RelationDetector
{
public:
    Eigen::MatrixXd cpts_;
    Eigen::Vector3d center_;

    std::vector<Eigen::MatrixXd> nodesCpts_;
    std::vector<Eigen::Vector3d> nodesCenter_;

    GlobalReflectionSymmDetector(Structure::Graph* g):RelationDetector(g)
    {
        std::vector<Eigen::Vector3d> cptsV;
        for ( int i = 0; i < (int) graph_->nodes.size(); ++i)
        {
            std::vector<Eigen::Vector3d> nodeCptsV;
            extractMainCpts( graph_->nodes[i], nodeCptsV, true);
            Eigen::MatrixXd nodeCptsM;
            vectorPts2MatrixPts(nodeCptsV, nodeCptsM);
            nodesCpts_.push_back(nodeCptsM);
            nodesCenter_.push_back( Eigen::Vector3d(nodeCptsM.col(0).mean(), nodeCptsM.col(1).mean(), nodeCptsM.col(2).mean()) );

            for ( std::vector<Eigen::Vector3d>::iterator it = nodeCptsV.begin(); it!= nodeCptsV.end(); ++it)
            {
                cptsV.push_back(*it);
            }
        }
        vectorPts2MatrixPts(cptsV, cpts_);

        //center_= this->graph_->bbox().center();// not extract for symm
        center_ = Eigen::Vector3d( cpts_.col(0).mean(), cpts_.col(1).mean(), cpts_.col(2).mean());
    }

    double detecting()
    {
        double meanScore(0.0), maxScore(0.0);
        Eigen::Vector3d normal = findReflectPlane();
        for ( int i = 0; i < (int) nodesCenter_.size(); ++i)
        {
            Eigen::Vector3d nc = nodesCenter_[i];
            Eigen::Vector3d nc1;
            reflectPoint(nc, center_, normal, nc1);
            double dist;
            int j = findNearestPart(nc1, graph_->nodes[i]->type(), dist);
            dist += std::abs(graph_->nodes[i]->bbox().diagonal().norm() - graph_->nodes[j]->bbox().diagonal().norm());///graph_->bbox().diagonal().norm();
            dist *= graph_->nodes[i]->bbox().diagonal().norm();
            /*Eigen::MatrixXd ptsout;
            reflectPoints(nodesCpts_[i], center_, normal, ptsout);
            dist = distanceBetween(ptsout, nodesCpts_[j]);*/
            meanScore += dist;
            if ( dist > maxScore)
            {
                maxScore = dist;
            }

        }
        meanScore = meanScore/nodesCenter_.size();
        meanScore = meanScore/graph_->bbox().diagonal().norm();
        meanScore = 1/(1+meanScore);

        maxScore = maxScore/graph_->bbox().diagonal().norm();
        maxScore = 1/(1+maxScore);
        return meanScore;
    }
private:
    void extractMainCpts( Structure::Node * n, std::vector<Eigen::Vector3d>& mcpts, bool bAll=false)
    {
        //if ( bAll)
        //{
        //	SurfaceMesh::Model * m1 = n->property["mesh"].value< QSharedPointer<SurfaceMeshModel> >().data();
        //	SurfaceMesh::Vector3VertexProperty pts1 = m1->vertex_coordinates();
        //	double* tmp;
        //	foreach(Vertex v1, m1->vertices())
        //	{
        //		tmp = pts1[v1].data();
        //		mcpts.push_back( Eigen::Vector3d(tmp[0], tmp[1], tmp[2]) );
        //	}
        //	return;
        //}

        if ( Structure::CURVE == n->type() )
        {
            Structure::Curve* c = dynamic_cast<Structure::Curve *>(n);
            if ( bAll)
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
            if ( bAll)
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
    Eigen::Vector3d findReflectPlane()
    {
        Eigen::Vector3d rnormal;
        double meanScore = std::numeric_limits<double>::max();//, maxScore;
        Eigen::Vector3d axis(0,0,1);
        Eigen::Vector3d initNormal(0,1,0), normal;
        double angleStep = 3.1415926*0.5;
        for ( int i = 0; i < (int) 2; ++i)
        {
            normal = rotatedVec(initNormal, i*angleStep, axis);
            double mean_dist, max_dist;
            errorOfRefSymmByCpts(center_, normal, mean_dist, max_dist);
            if ( mean_dist < meanScore)
            {
                meanScore = mean_dist;
                //maxScore = max_dist;
                rnormal = normal;
            }

#ifdef _OUTPUT_LOG
            if (logStream_.device())
            {
                logStream_ << "current plane's center: ["
                          << center_.x() << "," << center_.y() << "," << center_.z() << "], normal: ["
                          << normal.x() << "," << normal.y() << "," << normal.z() << "], error: " << max_dist << "\n";
            }
#endif
        }
        return rnormal;
    }
    //todo 遍历潜在对称面；目前：z冲上，所以就是过中心点的xy面或者yx面。
    int findNearestPart(Eigen::Vector3d &nc, QString & type, double& dist)
    {
        int no(0);
        dist = std::numeric_limits<double>::max();
        for ( int i = 0; i < (int) nodesCenter_.size(); ++i)
        {
            Eigen::Vector3d& nc1 = nodesCenter_[i];
            double tdist = (nc-nc1).norm();
            if ( graph_->nodes[i]->type() == type && tdist < dist)
            {
                dist = tdist;
                no = i;
            }
        }
        return no;
    }
    void errorOfRefSymmByCpts(const Eigen::Vector3d& center, const Eigen::Vector3d& normal, double &mean_dist, double &max_dist)
    {
        Eigen::MatrixXd ptsout;
        reflectPoints(cpts_, center, normal, ptsout);
        distanceBetween(cpts_, ptsout, mean_dist, max_dist);
    }
};
class PairRelationDetector : public RelationDetector
{
public:
    QVector<PairRelation> pairRelations_;

	PairRelationDetector(Structure::Graph* g):RelationDetector(g) { }

    void detecting()
    {
        int nNodes = graph_->nodes.size();
        for (int i=0; i<nNodes; ++i)
        {
            if ( graph_->nodes[i]->id.contains("null"))
                continue;
            double diam1 = graph_->nodes[i]->bbox().diagonal().norm();
            if ( diam1 < thDegeneratedNode_)
                continue;
        
            for (int j=i+1; j<nNodes; ++j)
            {
                if ( graph_->nodes[j]->id.contains("null"))
                    continue;
                double diam2 = graph_->nodes[j]->bbox().diagonal().norm();
                if ( diam2 < thDegeneratedNode_)
                    continue;

                bool hasTrans(false),hasRef(false);
                // compare radius // whether 2 part could be symmetry coarsely
                if ( (diam1/diam2) < thRadiusRadio_ && (diam2/diam1) < thRadiusRadio_ )
                {
                    hasTrans = has_trans_relation(i,j);//
                    // has_ref_relation(i,j); // it is redundant since we have trans relations
                    if ( !hasTrans)
                        hasRef = has_ref_relation(i, j);
                }
                if ( !hasTrans && !hasRef )
                {
                    isParalOrthoCoplanar(i,j);
                }
            }
        }
    }
private:

    bool isPairByName(QString& s1, QString& s2)
    {
        int idx = s1.indexOf(QRegExp("\\d"), 0);
        QString str1 = s1.left(idx);
        QString str2 = s2.left(idx);
        if ( str1 == str2)
            return true;
        else
            return false;
    }
    //bool has_rot_relation(int id1, int id2)
    //{
    //	Structure::Node * n1 = graph_->nodes[id1];
    //	Structure::Node * n2 = graph_->nodes[id2];
    //	if ( Structure::SHEET == n1->type() || Structure::SHEET == n2->type()) // jjcao todo, check planes have rot relation
    //		return false;

    //	Line_3 l1 = curve2line(n1);
    //	Line_3 l2 = curve2line(n2);

    //	Object result = intersection(l1, l2);
    //	if (const Point_3<Kernel> *ipoint = object_cast<Point_3<Kernel> >(&result))
    //	{
    //		Eigen::Vector3d center(ipoint->x(), ipoint->y(), ipoint->z());
    //
    //		Vector_3 v1 = l1;
    //		Vector_3 v2 = l2;
    //		Eigen::Vector3d vec1(v1.x(), v1.y(), v1.z());
    //		Eigen::Vector3d vec2(v2.x(), v2.y(), v2.z());
    //		vec1.normalize(); vec2.normalize();
    //		Eigen::Vector3d direction = vec1.cross(vec2);

    //		//////////////////////////
    //		double angle = asin( vec1.cross(vec2).norm() );
    //		double error;
    //		Structure::Node * n;
    //		if ( vec1.dot(vec2) > 0)
    //		{
    //			n = n2->clone();
    //			rotNodeCpts(n, center, direction, angle);
    //			error = distanceBetweenTransNodes(n1, n);
    //		}
    //		else
    //		{
    //			n = n1->clone();
    //			rotNodeCpts(n, center, direction, angle);
    //			error = distanceBetweenTransNodes(n2, n);
    //		}
    //
    //		//////////////////////////////
    //		if (error < thRot_)
    //		{
    //			PairRelation pairRel;
    //			pairRel.type = ROT;
    //			if ( vec1.dot(vec2) > 0)
    //			{
    //				pairRel.id1 = n1->id;
    //				pairRel.id2 = n2->id;
    //			}
    //			else
    //			{
    //				pairRel.id1 = n2->id;
    //				pairRel.id2 = n1->id;
    //			}
    ///*			pairRel.center = center;
    //			pairRel.direction = direction;*/
    //			pairRelations_.push_back(pairRel);
    //			return true;
    //		}
    //
    //	}
    //	else
    //	{
    //		return false;
    //	}
    //}
    bool has_ref_relation(int id1, int id2)
    {
        Structure::Node * n1 = graph_->nodes[id1];
        Structure::Node * n2 = graph_->nodes[id2];

        Eigen::Vector3d center, normal;
        findRefPlane(n1, n2, center,normal);

        Eigen::MatrixXd newverts2;
        transNodeMeshVerts(n2, center, normal, newverts2);
        double errorMesh = distanceBetweenTransNodesMesh(n1, newverts2);
        errorMesh = errorMesh / ((n1->bbox().diagonal().norm() + n2->bbox().diagonal().norm()) * 0.5);
        //////////////
        if ( isPairByName(n1->id, n2->id) )
        {
            //errorCpts *= 0.5;
            errorMesh *= 0.5;
        }
        /////////////

#ifdef _OUTPUT_LOG
            logStream_ <<"<(" << n1->id <<", " <<n2->id <<"), ref)> center: " << center[0] << ", " << center[1] <<", "<< center[2] << "\n"
                      <<"normal: " << normal[0] <<", " << normal[1] << ", " << normal[2] << "\n";
            logStream_ << "error mesh : " << errorMesh << "\n";
            logStream_ << "\n";
#endif
        if (errorMesh < thRefRadio_)
        {
            PairRelation pr(n1->id, n2->id, REF);
            pr.diameter = computePairDiameter(pr);
            pr.deviation = errorMesh;
            pairRelations_.push_back(pr);
            return true;
        }
        else
        {
            return false;
        }
    }

    bool has_trans_relation(int id1, int id2)
    {
        Structure::Node * n1 = graph_->nodes[id1];
        Structure::Node * n2 = graph_->nodes[id2];

        if ( n1->type().compare( n2->type()) )
            return false;

        Eigen::Vector3d transVec = graph_->nodes[id1]->center() - graph_->nodes[id2]->center();
        Eigen::MatrixXd newverts2;
        transNodeMeshVerts( n2, transVec,newverts2);
        double error = distanceBetweenTransNodesMesh(n1, newverts2);
        error = error / ((n1->bbox().diagonal().norm() + n2->bbox().diagonal().norm()) * 0.5);

        if (error < thTransRadio_)
        {
            PairRelation pr(n1->id, n2->id, TRANS);
            pr.trans_vec = transVec;
            pr.diameter = computePairDiameter(pr);
            pr.deviation = error;
            pairRelations_.push_back(pr);
            return true;
        }
        else
            return false;
    }

    void isParalOrthoCoplanar(int id1, int id2)
    {
        Structure::Node * n1 = graph_->nodes[id1];
        Structure::Node * n2 = graph_->nodes[id2];

        Vector_3 d1, d2;
        bool iscurve1 = node2direction(n1, d1);
        bool iscurve2 = node2direction(n2, d2);
        if ( iscurve1 == iscurve2 )
        {
            double error = errorOfParallel(d1, d2);

            if (error < thParal_)
            {
                PairRelation pr(n1->id, n2->id, PARALLEL);
                pr.deviation = error;
                pr.diameter = computePairDiameter(pr);
                pairRelations_.push_back(pr);
            }
            else
            {
                double error = errorOfOrthogonal(d1, d2);

                if ( error < thOthog_)
                {
                    PairRelation pr(n1->id, n2->id, ORTHOGONAL);
                    pr.deviation = error;
                    pr.diameter = computePairDiameter(pr);
                    pairRelations_.push_back( pr );
                }
                if (iscurve1 && iscurve2 )
                {
                    Line_3 l1 = curve2line(n1), l2 = curve2line(n2);
                    error = squared_distance(l1, l2);

                    if ( error < thCopla_)
                    {
                        PairRelation pr(n1->id, n2->id, COPLANAR);
                        createPlane(n1,n2, pr.point, pr.normal);
                        pr.deviation = error;
                        pr.diameter = computePairDiameter(pr);
                        pairRelations_.push_back( pr );
                    }
                }
            }
        }
        else// curve & sheet
        {
            double error = errorOfParallel(d1, d2);

            if (error < thOthog_)
            {
                PairRelation pr(n1->id, n2->id, ORTHOGONAL);
                pr.deviation = error;
                pr.diameter = computePairDiameter(pr);
                pairRelations_.push_back(pr);
            }
            else
            {
                double error;
                Vector3 point, normal;
                if ( iscurve1 )
                {
                    sheet2plane(dynamic_cast<Structure::Sheet *>(n2), point, normal);
                    error = errorOfLineInPlane( curve2segment(n1), point, normal);
                }
                else
                {
                    sheet2plane(dynamic_cast<Structure::Sheet *>(n1), point, normal);
                    error = errorOfLineInPlane( curve2segment(n2), point, normal);
                }

                if ( error < thCopla_)
                {
                    PairRelation pr(n1->id, n2->id, COPLANAR);
                    pr.point = point;
                    pr.normal = normal;
                    pr.deviation = error;
                    pr.diameter = computePairDiameter(pr);
                    pairRelations_.push_back(pr);
                }
            }
        }
    }
};
class GroupRelationDetector : public RelationDetector
{
public:
    QVector<GroupRelation> groupRelations_;

    GroupRelationDetector(Structure::Graph* g):RelationDetector(g)  {    }
    // input: prRelations
    // output
    // 1. prRelations, after some of them are removed since they have been added into group relations
    // 2. groupRelations_
    // 3. groupScore
    // 4. pairScore
    void detecting(QVector<PairRelation>& prRelations, double& groupScore, double& pairScore)
    {
        detectSymmGroupByRefPair( prRelations);
        detectSymmGroupByTransPair( prRelations);
        detectCoplanarGroup( prRelations);
        mergeCoplanarGroup();
        removeRedundantGroup();

        //////////////////
        std::vector<double> score;
        for ( int i = 0; i < (int) groupRelations_.size(); ++i)
        {
            GroupRelation& gr = groupRelations_[i];
            computeGroupDeviationByCpts(gr,gr);
            score.push_back( computeScore(gr, gr));
        }
        groupScore = std::accumulate(score.begin(),score.end(), 0.0);
        if (groupRelations_.empty())
            groupScore = 0;
        else
            groupScore = groupScore/(graph_->bbox().diagonal().norm()*groupRelations_.size());

        ///////////////////
        score.clear();
        for ( int i = 0; i < (int) prRelations.size(); ++i)
        {
            PairRelation& pr = prRelations[i];
            double ss = computeScore(pr, pr);
            score.push_back( ss);
        }
        pairScore = std::accumulate(score.begin(),score.end(), 0.0);
        if (prRelations.empty())
            pairScore = 0;
        else
            pairScore = pairScore/(graph_->bbox().diagonal().norm()*prRelations.size());
    }
protected:
    void removeRedundantGroup()
    {
        for ( int i = 0; i < (int) groupRelations_.size(); ++i)
        {
            groupRelations_[i].tag = false;
        }
        for ( int i = 0; i < (int) groupRelations_.size(); ++i)
        {
            if ( groupRelations_[i].tag )
                continue;

            for ( int j = i+1; j < (int) groupRelations_.size(); ++j)
            {
                if ( isSubgroup(groupRelations_[i].ids, groupRelations_[j].ids) )
                    groupRelations_[j].tag = true;
            }
        }
        groupRelations_.erase( std::remove_if(groupRelations_.begin(), groupRelations_.end(), isTagedGr), groupRelations_.end() );
    }
    // merge coplanar group
    void mergeCoplanarGroup()
    {
        for ( int i = 0; i < (int) groupRelations_.size(); ++i)
        {
            groupRelations_[i].tag = false;
        }
        for ( int i = 0; i < (int) groupRelations_.size(); ++i)
        {
            if ( COPLANAR != groupRelations_[i].type || groupRelations_[i].tag)
                continue;

            Vector3 pt1 = groupRelations_[i].point;
            Vector3 normal1 = groupRelations_[i].normal;
            QSet<QString> ids;
            vectorId2SetId(groupRelations_[i].ids, ids);

            for ( int j = i+1; j < (int) groupRelations_.size(); ++j)
            {
                if ( COPLANAR != groupRelations_[j].type || groupRelations_[j].tag)
                    continue;

                Vector3 pt2 = groupRelations_[j].point;
                Vector3 normal2 = groupRelations_[j].normal;
                double err = errorOfCoplanar(pt1, normal1, pt2, normal2);

                if ( err < thCoplaGroup_)
                {
                    groupRelations_[j].tag = true;
                    vectorId2SetId(groupRelations_[j].ids, ids);
                }
            }

            groupRelations_[i].ids.clear();
            setId2VectorId(ids, groupRelations_[i].ids);
        }

        groupRelations_.erase( std::remove_if(groupRelations_.begin(), groupRelations_.end(), isTagedGr), groupRelations_.end() );
    }
    // is ids2 is a subgroup of ids1.
    bool isSubgroup(QVector<QString>& ids1, QVector<QString>& ids2)
    {
        int count(0);
        for ( int i = 0; i < (int) ids2.size(); ++i)
        {
            for ( int j = 0; j < (int) ids1.size(); ++j)
            {
                if ( ids2[i] == ids1[j] )
                {
                    ++count;
                    break;
                }
            }
        }
        if ( ids2.size() == count)
            return true;
        else
            return false;
    }
    bool isIntersected(QSet<QString>& ids, PairRelation& pr2)
    {
        for ( QSet<QString>::iterator it = ids.begin(); it != ids.end(); ++it)
        {
            if ( *it == pr2.id1 || *it == pr2.id2)
                return true;
        }
        return false;
    }
    QString symmTypeOfTansPair(PairRelation& pr1)
    {
        QString result(NONE);
        Structure::Node* n1 = graph_->getNode(pr1.id1);
        Vector_3 d1;
        bool iscurve1 = node2direction(n1, d1);
        if ( iscurve1 )
        {
            result = AXIS_SYMMETRY;
        }
        else
        {
            result = REF_SYMMETRY;
        }

        return result;
    }

    // axis & ref symmetry
    void detectSymmGroupByTransPair(QVector<PairRelation>& prRelations)
    {
        int nPrs = prRelations.size();
        for (int i=0; i<nPrs; ++i)
            prRelations[i].tag = false;

        ///////////
        // build group from pair
        for (int i=0; i<nPrs; ++i)
        {
            PairRelation& pr1 = prRelations[i];
            if ( pr1.tag || TRANS.compare(pr1.type)) continue;

            QSet<QString> ids;
            GroupRelation gr;
            gr.type = symmTypeOfTansPair(pr1);
            if ( NONE != gr.type)
            {
                ids.insert( pr1.id1);
                ids.insert( pr1.id2);
                pr1.tag = true;
            }
            else
                continue;

            ///////////////
            for (int j=i+1; j<nPrs; ++j)
            {
                PairRelation& pr2 = prRelations[j];
                if ( pr2.tag || TRANS.compare(pr2.type)) continue;

                if ( isIntersected(ids, pr2) )
                {
                    QString type2 = symmTypeOfTansPair(pr2);
                    if ( NONE != type2)
                    {
                        if ( REF_SYMMETRY == gr.type)
                        {
                            if ( std::abs( pr1.trans_vec.norm()-pr2.trans_vec.norm() ) >
                                graph_->getNode(pr1.id1)->bbox().diagonal().norm()*thRefGroup_*2 )
                            {
                                continue;
                            }
                        }
                        ids.insert(pr2.id1);
                        ids.insert(pr2.id2);
                        pr2.tag = true;
                    }
                }
            }

            ///////////////
            computeTransGroupInfo(gr, ids);
            gr.deviation = computeAxisSymmetryGroupDeviation(gr, true);
            if ( gr.deviation / graph_->bbox().diagonal().norm() < thAxisGroup_)
            {
                groupRelations_.push_back(gr);
                continue;
            }
            else
            {
                gr.type = REF_SYMMETRY;
                gr.deviation = computeRefSymmetryGroupDeviation(gr, true);
                if ( gr.deviation / graph_->bbox().diagonal().norm() < thRefGroup_)
                {
                    groupRelations_.push_back(gr);
                }
            }
        }
        ////////////// remove pairs have been put into groups
        prRelations.erase( std::remove_if(prRelations.begin(), prRelations.end(), isTaged), prRelations.end() );
    }
    // axis & ref symmetry
    void detectSymmGroupByRefPair(QVector<PairRelation>& prRelations)
    {
        int nPrs = prRelations.size();
        for (int i=0; i<nPrs; ++i)
            prRelations[i].tag = false;

        double dotVal1(0.0), dotVal2(0.0);
        ///////////
        // build group from REF pair
        for (int i=0; i<nPrs; ++i)
        {
            PairRelation& pr1 = prRelations[i];
            if ( pr1.tag || REF.compare(pr1.type)) continue;

            QSet<QString> ids;
            GroupRelation gr;
            ids.insert( pr1.id1);			ids.insert( pr1.id2);
            pr1.tag = true;
            Structure::Node* n1 = graph_->getNode(pr1.id1);
            Structure::Node* n2 = graph_->getNode(pr1.id2);
            Vector_3 v1, v2;
            node2direction(n1,v1); node2direction(n2,v2);
            dotVal1 = std::abs(dot(v1,v2));
            if ( dotVal1 > thAxisDeviationRadio_)
            {
                if ( dot(v1,v2) < 0)
                    gr.direction = v1 -v2;
                else
                    gr.direction = v1+v2;
            }
            else
            {
                gr.direction = cross_product(v1, v2);
            }
            gr.direction = gr.direction/sqrt(gr.direction.squaredNorm());

            ///////////////
            for (int j=i+1; j<nPrs; ++j)
            {
                PairRelation& pr2 = prRelations[j];
                if ( pr2.tag || REF.compare(pr2.type)) continue;

                if ( isIntersected(ids, pr2) )
                {
                    n1 = graph_->getNode(pr2.id1);
                    n2 = graph_->getNode(pr2.id2);
                    node2direction(n1,v1); node2direction(n2,v2);
                    Vector_3 dir;
                    if ( dotVal1 > thAxisDeviationRadio_)
                    {
                        dir = v1 + v2;
                    }
                    else
                    {
                        dotVal2 = std::abs( dot(v1,v2) );
                        if ( dotVal2 > thAxisDeviationRadio_)
                            continue;
                        else
                            dir = cross_product(v1, v2);
                    }
                    dir = dir/sqrt(dir.squaredNorm());

                    double tmp = std::abs( dot(gr.direction, dir) );
                    if ( tmp > thAxisDeviationRadio_)
                    {
                        if ( dot(gr.direction, dir) < 0)
                            gr.direction = gr.direction - dir;
                        else
                            gr.direction = gr.direction + dir;
                        gr.direction = gr.direction/sqrt(gr.direction.squaredNorm());

                        ids.insert(pr2.id1);	ids.insert(pr2.id2);
                        pr2.tag = true;
                    }
                }
            }

            ///////////////
            Eigen::Vector3d center(0,0,0);
            for ( QSet<QString>::iterator it = ids.begin(); it != ids.end(); ++it)
            {
                Structure::Node* n1 = graph_->getNode(*it);
                center = center + n1->center();
                gr.ids.push_back(*it);
            }
            center = center / ids.size();
            gr.center = Point_3(center.x(), center.y(), center.z());
            gr.diameter = computeGroupDiameter(gr);

            ////////////////
            gr.deviation = computeAxisSymmetryGroupDeviation(gr, true);
            gr.type = AXIS_SYMMETRY;
            if ( gr.deviation / graph_->bbox().diagonal().norm() < thAxisGroup_)
            {
                groupRelations_.push_back(gr);
            }

            //else
            //{
                gr.type = REF_SYMMETRY;
                gr.deviation = computeRefSymmetryGroupDeviation(gr,true);
                if ( gr.deviation / graph_->bbox().diagonal().norm() < thRefGroup_)
                {
                    groupRelations_.push_back(gr);
                }
            //}

        }
        ////////////// remove pairs have been put into groups
        prRelations.erase( std::remove_if(prRelations.begin(), prRelations.end(), isTaged), prRelations.end() );
    }


    void detectCoplanarGroup(QVector<PairRelation>& prRelations)
    {
        int nPrs = prRelations.size();
        for (int i=0; i<nPrs; ++i)
        {
            prRelations[i].tag = false;
        }

        ///////////
        for (int i=0; i<nPrs; ++i)
        {
            PairRelation& pr1 = prRelations[i];
            if (pr1.tag || COPLANAR.compare(pr1.type)) continue;

            GroupRelation gr;
            QSet<QString> ids;
            gr.type = COPLANAR;
            ids.insert(pr1.id1);
            ids.insert(pr1.id2);
            gr.point = pr1.point;
            gr.normal = pr1.normal;
            gr.deviation = pr1.deviation;

            for (int j=i+1; j<nPrs; ++j)
            {
                PairRelation& pr2 = prRelations[j];
                if (pr2.tag || COPLANAR.compare(pr2.type)) continue;

                double error = errorOfCoplanar(pr1.point, pr1.normal, pr2.point, pr2.normal);

                if ( error < thCoplaGroup_)
                {
                    ids.insert(pr2.id1);
                    ids.insert(pr2.id2);
                    pr2.tag = true;
                    gr.deviation += error;
                }
            }

            ///////////////
            gr.deviation = gr.deviation/(ids.size()-1);
            if ( gr.deviation < thCoplaGroup_)
            {
                for ( QSet<QString>::iterator it = ids.begin(); it!=ids.end(); ++it)
                {
                    gr.ids.push_back(*it);
                }
                gr.diameter = computeGroupDiameter(gr);
                groupRelations_.push_back(gr);
                pr1.tag = true;
            }
        }

        ////////////// remove pairs have been put into groups
        prRelations.erase( std::remove_if(prRelations.begin(), prRelations.end(), isTaged), prRelations.end() );
    }
};
class PairRelationTracer : public RelationDetector
{
public:
    double diagonalLen_;
    PairRelationTracer(Structure::Graph* g, double diagonalLen):RelationDetector(g),diagonalLen_(diagonalLen){    }
    double detecting(QVector<PairRelation>& prs,QVector<PART_LANDMARK> &corres, int ith)
    {
        std::vector<double> score;
        QVector<PairRelation> cprs;
        int numCurrPr(0);
        for ( int i = 0; i < (int) prs.size(); ++i)
        {
            PairRelation pr = prs[i];
            PairRelation cpr = findCorrespondencePair(this->graph_,pr,corres, 0==ith);
            if ( !isDegeneratePair(cpr))
            {
                ++numCurrPr;
            }
            cprs.push_back(cpr);

            double ss = computeScore(cpr, pr);
            score.push_back( ss);
        }
        double tmp = std::accumulate(score.begin(),score.end(), 0.0);
        if ( numCurrPr == 0)
            tmp = 0;
        else
            tmp = tmp/(diagonalLen_*numCurrPr);

        return tmp;
    }
private:
    void isParalOrthoCoplanar(Structure::Node * n1, Structure::Node * n2,PairRelation &cpr)
    {
        Vector_3 d1, d2;
        bool iscurve1 = node2direction(n1, d1);
        bool iscurve2 = node2direction(n2, d2);
        double err(0.0);
        if ( iscurve1 == iscurve2 )
        {
            if ( cpr.type == PARALLEL)
            {
                err = errorOfParallel(d1, d2);

            }
            else if ( cpr.type == ORTHOGONAL)
            {
                err = errorOfOrthogonal(d1, d2);
            }
        }
        else// curve & sheet
        {
            if ( cpr.type == ORTHOGONAL)
            {
                err = errorOfParallel(d1, d2);
            }
        }

        cpr.deviation = err;
        cpr.id1 = n1->id;
        cpr.id2 = n2->id;
        cpr.diameter = computePairDiameter(cpr);
    }

    std::vector<Structure::Node*> filterNodesByDiameter(std::vector<Structure::Node*>& invec, double thMinDiamater)
    {
        std::vector<Structure::Node*> outvec;
        double diameter;
        for ( std::vector<Structure::Node*>::iterator it = invec.begin(); it!=invec.end(); ++it)
        {
            if ( *it == NULL)
                continue;

            diameter = (*it)->bbox().diagonal().norm();
            if ( diameter > thMinDiamater)
            {
                outvec.push_back(*it);
            }
        }
        return outvec;
    }
    // trace pair constraints: ORTHOGONAL or PARALLEL
    PairRelation findCorrespondencePair(Structure::Graph *graph, PairRelation &pr, QVector<PART_LANDMARK>& corres,bool bSource)
    {
        PairRelation cpr;
        cpr.diameter = 0;
        std::vector<Structure::Node*> tmpNodes1 = findNodes(pr.id1, graph, corres, bSource);
        std::vector<Structure::Node*> tmpNodes2 = findNodes(pr.id2, graph, corres, bSource);
        std::vector<Structure::Node*> nodes1 = filterNodesByDiameter(tmpNodes1, thDegeneratedNode_);
        std::vector<Structure::Node*> nodes2 = filterNodesByDiameter(tmpNodes2, thDegeneratedNode_);

        ////////////////
        cpr.deviation = 0;
        cpr.type = pr.type;
        if ( nodes1.empty() || nodes2.empty())
        {
            setInfoForDegeneratedPair(cpr);
            return cpr;
        }

        /////////////////////////
        for ( int i = 0; i < (int) nodes1.size(); ++i)
        {
            for ( int j = 0; j < (int) nodes2.size(); ++j)
            {
                isParalOrthoCoplanar(nodes1[i], nodes2[j], cpr);
            }
        }
        return cpr;
    }
};

class GroupRelationTracer : public RelationDetector
{
public:
    double diagonalLen_;
	GroupRelationTracer(Structure::Graph* g, double diagonalLen):RelationDetector(g),diagonalLen_(diagonalLen){}

    // return score of current shape relative to ith shape
    // ith takes two values: ith = 0 means source, 1 means target.
    double detecting(QVector<GroupRelation>& grs,QVector<PART_LANDMARK> &corres, int ith)
    {
        std::vector<double> score;
        int numCurrGr (0);
        for ( int j = 0; j < (int) grs.size(); ++j)
        {
            GroupRelation gr = grs[j];
            GroupRelation cgr = findCorrespondenceGroup(this->graph_,gr,corres, 0==ith);
            if ( !isDegenerateGroup(cgr) )
            {
                ++numCurrGr;
                //if ( cgr.deviation <
            }

            /*if ( cgr.type == REF_SYMMETRY)
            {*/
                double ss = computeScore(cgr, gr);
                score.push_back( ss);
            //}
        }
        double tmp = std::accumulate(score.begin(),score.end(), 0.0);
        if ( numCurrGr == 0)
            tmp = 0;
        else
            tmp = tmp/(diagonalLen_*numCurrGr);
        return tmp;
    }
protected:
    GroupRelation findCorrespondenceGroup(Structure::Graph *graph, GroupRelation &gr,QVector<PART_LANDMARK>& corres,bool bSource)
    {
        GroupRelation cgr;
        std::vector<double> nodeDiameter;
        double diameter, maxDiameter(0.0);
        int maxIdx(0);

        std::vector<Structure::Node*> tmpNodes;
        for ( int i = 0; i < (int) gr.ids.size(); ++i)
        {
            //Structure::Node * tnode = tgraph->getNode( snode->property["correspond"].toString() );
            std::vector<Structure::Node*> nodes = findNodes(gr.ids[i], graph, corres, bSource);
            for ( int j = 0; j < (int) nodes.size(); ++j)
            {
                Structure::Node* n = nodes[j];
                if ( NULL == n) continue;
                diameter = n->bbox().diagonal().norm();
                if ( diameter < thDegeneratedNode_) continue;

                tmpNodes.push_back(n);
                nodeDiameter.push_back(diameter);
                if ( diameter > maxDiameter)
                {
                    maxDiameter = diameter;
                    maxIdx = tmpNodes.size()-1;
                }
            }
        }

        //////////////	use max node as the first node, the following will be transformed into it, & compare with it.
        cgr.deviation = 0;
        if ( tmpNodes.size()<2 )
        {
            setInfoForDegeneratedGroup(cgr);
            return cgr;
        }

        for ( int i = maxIdx; i < (int) tmpNodes.size(); ++i)
        {
            Structure::Node* n = tmpNodes[i];
            cgr.ids.push_back(n->id);
        }
        for ( int i = 0; i < (int) maxIdx; ++i)
        {
            Structure::Node* n = tmpNodes[i];
            cgr.ids.push_back(n->id);
        }
        cgr.diameter = computeGroupDiameter(cgr);

        //////////////////////
        computeGroupDeviationByCpts(cgr, gr);
        return cgr;
    }
};
// trace relation from g1_ to g2_
class RelationComparison
{
public:
    Structure::Graph* g1_;
    Structure::Graph* g2_;
    int paralSymmKept_, axisSymmKept_, copKept_, paralKept_, orthoKept_;
    int paralSymmAdd_, axisSymmAdd_, copAdd_, paralAdd_, orthoAdd_;
    RelationComparison(Structure::Graph* g1, Structure::Graph* g2)
        :g1_(g1),g2_(g2),
        paralSymmKept_(0), axisSymmKept_(0), copKept_(0), paralKept_(0), orthoKept_(0),
        paralSymmAdd_(0), axisSymmAdd_(0), copAdd_(0), paralAdd_(0), orthoAdd_(0)
    {}
    double getRelationWeight(PairRelation& pr, Structure::Graph* g)
    {
        Eigen::AlignedBox3d bbox1 = g->getNode(pr.id1)->bbox();
        Eigen::AlignedBox3d bbox2 = g->getNode(pr.id2)->bbox();
        Eigen::AlignedBox3d bbox = bbox1.merged(bbox2);
        //return bbox.volume();
        return bbox.diagonal().norm()/g->bbox().diagonal().norm();
    }
    double getRelationWeight(GroupRelation& gr, Structure::Graph* g)
    {
        Eigen::AlignedBox3d bbox;
        for ( QVector<QString>::iterator it = gr.ids.begin(); it!=gr.ids.end(); ++it)
        {
            Eigen::AlignedBox3d bbox1 = g->getNode(*it)->bbox();
            bbox = bbox.merged(bbox1);
        }
        //return bbox.volume();
        return bbox.diagonal().norm()/g->bbox().diagonal().norm();
    }
    // T is PairRelation or GroupRelation
    template<class T>
    QVector<double> relationKept(QVector<T>& prs1, QVector<T>& prs2)
    {
        for ( int i = 0; i < (int) prs2.size(); ++i)
        {
            prs2[i].tag = false;
        }

        QVector<double> score(prs1.size(), 0.0);
        int i(0);
        foreach(T pr1, prs1)
        {
            bool bFound(false);
            for ( int j = 0; j < (int) prs2.size(); ++j)
            {
                T& pr2 = prs2[j];
                if ( !pr2.tag && pr1.equal(pr2) )
                {
                    bFound = true;
                    pr2.tag = true;
                    break;
                }
            }
            if ( bFound)
            {
                if ( pr1.type == AXIS_SYMMETRY ) ++axisSymmKept_;
                else if ( pr1.type == COPLANAR ) ++copKept_;
                else if ( pr1.type == PARALLEL ) ++paralKept_;
                else ++orthoKept_;

                score[i] = getRelationWeight(pr1, g1_);
            }
            ++i;
        }
        return score;
    }
    template<class T>
    QVector<double> relationAdd(QVector<T>& prs1, QVector<T>& prs2)
    {
        for ( int i = 0; i < (int) prs1.size(); ++i)
        {
            prs1[i].tag = false;
        }

        QVector<double> score(prs2.size(), 0.0);
        int i(0);
        foreach(T pr2, prs2)
        {
            bool bFound(false);
            for ( int j = 0; j < (int) prs1.size(); ++j)
            {
                T& pr1 = prs1[j];
                if ( !pr1.tag && pr1.equal(pr2) )
                {
                    bFound = true;
                    pr2.tag = true;
                    break;
                }
            }
            if ( !bFound)
            {
                if ( pr2.type == AXIS_SYMMETRY ) ++axisSymmAdd_;
                else if ( pr2.type == COPLANAR ) ++copAdd_;
                else if ( pr2.type == PARALLEL ) ++paralAdd_;
                else ++orthoAdd_;

                score[i] = getRelationWeight(pr2, g2_);
            }
            ++i;
        }
        return score;
    }
};
