#include "RelationDetector.h"

QTextStream& operator << (QTextStream& os, const PairRelation& pr)
{    
    os << "Pair <" << pr.id1 << ", " << pr.id2 << ": " << pr.type << ">\n";
	
	os << "Trans vector: " << pr.trans_vec.x() << ", " << pr.trans_vec.y() << ", " << pr.trans_vec.z() << "\n";

	Vector3 normal = pr.normal;
	Vector3 point = pr.point;
	os << "normal of plane: " << normal.x() << ", " << normal.y() << ", " << normal.z() << "\n";
	os << "point of plane: " << point.x() << ", " << point.y() << ", " << point.z() << "\n";

	os << "Diameter: " << pr.diameter << "\n";
	os << "Deviation: " << pr.deviation << "\n\n";
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
	os << "Center: " << gr.center.x() << ", " << gr.center.y() << ", " << gr.center.z() << "\n";
	os << "Direction: " << gr.direction.x() << ", " << gr.direction.y() << ", " << gr.direction.z() << "\n";
	
	os << "normal of ref plane: " << gr.refPlane.a() << ", " << gr.refPlane.b() << ", " << gr.refPlane.c() << ", " << gr.refPlane.d() << "\n";

	Vector3 normal = gr.normal;
	Vector3 point = gr.point;
	os << "normal of plane: " << normal.x() << ", " << normal.y() << ", " << normal.z() << "\n";
	os << "point of plane: " << point.x() << ", " << point.y() << ", " << point.z() << "\n";

	os << "Diameter: " << gr.diameter << "\n";
	os << "Deviation: " << gr.deviation << "\n\n";
    return os;
}

void mini(Vector3& vin, Vector3& vout)
{
	for ( int i = 0; i < 3; ++i)
	{
		if (vin[i] > vout[i])  vin[i] = vout[i];
	}
}
void maxm(Vector3& vin, Vector3& vout)
{
	for ( int i = 0; i < 3; ++i)
	{
		if (vin[i] < vout[i])  vin[i] = vout[i];
	}
}
Segment_3 curve2segment(Structure::Node * n)
{
	const Vector3& bpn = n->controlPoint(0);
	const Vector3& epn = n->controlPoint(n->numCtrlPnts()-1);

	Segment_3 seg(Point_3(bpn.x(),bpn.y(),bpn.z()), Point_3(epn.x(),epn.y(),epn.z()));
	return seg;
}
Eigen::Vector3d curve2vectorNormalized(Structure::Node * n)
{
	const Vector3& bpn = n->controlPoint(0);
	const Vector3& epn = n->controlPoint(n->numCtrlPnts()-1);
	Eigen::Vector3d vec = bpn - epn;
	vec.normalize();
	return vec;
}
Line_3 curve2line(Structure::Node * n)
{
	return Line_3(curve2segment(n));
}
std::vector<Point_3> sheet2rect(Structure::Node * n)
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
void vectorId2SetId(QVector<QString>& vec, QSet<QString>& set)
{
	for ( QVector<QString>::iterator it = vec.begin(); it!=vec.end(); ++it)
	{
		set.insert(*it);
	}
}
void setId2VectorId(QSet<QString>& set, QVector<QString>& vec)
{
	for ( QSet<QString>::iterator it = set.begin(); it!=set.end(); ++it)
	{
		vec.push_back(*it);
	}
}
void sheet2plane(Structure::Sheet * s, Vector3& pos, Vector3& normal)
{
	std::vector<Vector3> nf = noFrame();
	s->get(Vector4d(0.5,0.5,0,0), pos, nf);
	normal = nf[2];
}
bool node2direction(Structure::Node * n, Vector_3& result)
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
bool typeLessThan(const PairRelation &pr1, const PairRelation &pr2)
{
	return pr1.type < pr2.type;
}
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
	out << "    REF: " << countByType(prs,REF) << "\n";
	out << "    PARALLEL: " << countByType(prs,PARALLEL) << "\n";
	out << "    ORTHOGONAL: " << countByType(prs,ORTHOGONAL) << "\n";
	out << "    COPLANAR: " << countByType(prs,COPLANAR) << "\n\n";

	foreach(PairRelation pr, prs)
	{
		out << i << " " << pr.type << ": " << pr.id1 << ", " << pr.id2 << ", ";
		if ( pr.type == TRANS)
			out << "Trans: ["<<pr.trans_vec[0]<<", "<<pr.trans_vec[1]<<", "<<pr.trans_vec[2]<< "]\n";
		else if ( pr.type == COPLANAR)
			out << "Plane: [" << pr.point.x()<<", "<<pr.point.y()<<", " <<pr.point.z()<<", "
							<< pr.normal.x()<<", "<<pr.normal.y()<<", " <<pr.normal.z()<<"]\n";
		else if ( pr.type == PARALLEL || pr.type == ORTHOGONAL)
		{
			out << "deviation: "<<  pr.deviation << "\n";
		}
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
	foreach(GroupRelation gr, grs){
		out << i << " <group-" << gr.type << ">: ";
		if ( gr.type == COPLANAR)
			out << gr.point.x() << "," << gr.point.y() << "," << gr.point.z() << ","
			    << gr.normal.x() << "," << gr.normal.y() << "," << gr.normal.z() << "\n";
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
bool isInGroupById(QString id, QVector<QString>& group)
{
	foreach(QString str, group)
	{
		if (0 == id.compare(str))
			return true;
	}
	return false;
}
//bool isCoplanar(Plane_3 &pp, QVector<Plane_3> &planes,double planarThreshold)
//{
//	bool result(false);
//	foreach(Plane_3 p, planes)
//	{
//		double error = errorOfCoplanar(pp, p);
//		if ( error < planarThreshold )
//			return true;
//	}
//	return result;
//}
QString group2str(QVector<QString> &group)
{
	QString str;
	foreach(QString istr, group)
	{
		str.append(istr);
	}
	return str;
}
bool isInGroups(QVector<QString> &group, Structure::NodeGroups &planarGroups)
{
	QString str = group2str(group);
	foreach (QVector<QString> g, planarGroups)
	{
		QString gstr = group2str(g);
		if ( 0 == gstr.compare(str))
			return true;
	}
	return false;
}
double errorOfLineInPlane(Segment_3 &l, Vector3& point, Vector3& normal)
{
	Plane_3 plane( Point_3(point.x(), point.y(), point.z()), 
				  Vector_3(normal.x(), normal.y(), normal.z()) );		
	double dist1 = squared_distance(plane, l.point(0));
	double dist2 = squared_distance(plane, l.point(1));
	return sqrt(dist1)+sqrt(dist2);
}

void saveScores( QVector<double>& scoreKeptGr, /*QVector<double>& scoreAddGr, */ QVector<double>& scoreKeptPr,/*QVector<double>& scoreAddPr, */ QTextStream& out )
{
	out << "    basic group kept score: " << std::accumulate(scoreKeptGr.begin(),scoreKeptGr.end(),0.0) << "\n";
	//out << "    basic group add score: " << std::accumulate(scoreAddGr.begin(),scoreAddGr.end(),0.0) << "\n";
	double scoreFinal(0.0), scoreGroup;
	scoreFinal += 4.0 * std::accumulate(scoreKeptGr.begin(),scoreKeptGr.end(),0.0);
	//scoreFinal += 0.5 * std::accumulate(scoreAddGr.begin(),scoreAddGr.end(),0.0);
	scoreGroup = scoreFinal;
	out << "    weighted group score: " << scoreGroup << "\n";

	scoreFinal += 1.0 * std::accumulate(scoreKeptPr.begin(),scoreKeptPr.end(),0.0);
	//scoreFinal += 0.1 * std::accumulate(scoreAddPr.begin(),scoreAddPr.end(),0.0);
	out << "    basic pair kept score: " << std::accumulate(scoreKeptPr.begin(),scoreKeptPr.end(),0.0) << "\n";
	//out << "    basic pair add score: " << std::accumulate(scoreAddPr.begin(),scoreAddPr.end(),0.0) << "\n";
	out << "    weighted pair score: " << scoreFinal - scoreGroup<< "\n";

	out << "    final score: "  << scoreFinal << "\n";
}
