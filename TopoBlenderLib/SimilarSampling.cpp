#include "SimilarSampling.h"
#include "StructureGlobal.h" // to-do: should be independent

// Helpers
static inline Scalar deg_to_rad(const Scalar& _angle){ return M_PI*(_angle/180.0); }
static inline Vec3d barycentric(Vec3d p, Vec3d a, Vec3d b, Vec3d c){
	Vec3d v0 = b - a, v1 = c - a, v2 = p - a;
	double d00 = dot(v0, v0); double d01 = dot(v0, v1);
	double d11 = dot(v1, v1); double d20 = dot(v2, v0);
	double d21 = dot(v2, v1); double denom = d00 * d11 - d01 * d01;
	double v = (d11 * d20 - d01 * d21) / denom;
	double w = (d00 * d21 - d01 * d20) / denom;
	double u = 1.0 - v - w;
	return Vec3d(u,v,w);
}
static inline bool isValidBaryCoord(Vec3d coord){
	if(	coord[0] < 0 || coord[1] < 0 || coord[2] < 0 ||
		coord[0] > 1 || coord[1] > 1 || coord[2] > 1) return false;
	return true;
}

QVector<Vector3> SimilarSampler::FaceSamples(SurfaceMeshModel * m, int sampleNum, QVector<Vector3> & samplesNormals, double * avgSpacing)
{
    QVector<Vector3> samples;

    // Compute face areas
    SurfaceMeshHelper h( m );
    h.computeFaceAreas();
    ScalarFaceProperty farea = m->face_property<Scalar>(FAREA);
	   
	Scalar area = 0;
	foreach(Face f, m->faces()) area += farea[f];

	// Compute edge lengths
	ScalarEdgeProperty elength = h.computeEdgeLengths();

    m->update_face_normals();
    Vector3FaceProperty fnormals = m->get_face_property<Vector3>(FNORMAL);

    Scalar samplePerAreaUnit = sampleNum / area;

    // Mesh points
    Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);

    // Similar Triangles sampling
    foreach(SurfaceMeshModel::Face f, m->faces())
    {
		// Collect vector of triangle points, and map to vertices
        std::vector<Vector3> triangle, virtualTri;
		QMap<SurfaceMesh::Vertex, int> verts;
        Surface_mesh::Vertex_around_face_circulator vit = m->vertices(f),vend=vit;
        do{ verts[vit] = triangle.size(); triangle.push_back(points[vit]);  } while(++vit != vend);
		virtualTri = triangle;

		// Force virtual triangle to be isosceles
		{
			QMap<SurfaceMesh::Halfedge,double> edgeMap;

			// Classify edges by their lengths
			Surface_mesh::Halfedge_around_face_circulator hj(m, f), hend = hj;
			do{ edgeMap[hj] = elength[m->edge(hj)]; } while (++hj != hend);
			QList< QPair<double,SurfaceMesh::Halfedge> > edges = sortQMapByValue(edgeMap);
			SurfaceMesh::Halfedge S = edges.at(0).second, M = edges.at(1).second, L = edges.at(2).second;

			SurfaceMesh::Vertex vP = m->to_vertex(m->next_halfedge(L));
			SurfaceMesh::Vertex v0 = (vP == m->to_vertex(S)) ? m->from_vertex(S) : m->to_vertex(S);
			SurfaceMesh::Vertex vM = (vP == m->to_vertex(M)) ? m->from_vertex(M) : m->to_vertex(M);
			Vec3d deltaS = (points[vP] - points[v0]).normalized();
			Vec3d deltaL = (points[vM] - points[v0]).normalized();

			// Push vertex towards triangle with two equal edges
			virtualTri[ verts[vP] ] = points[v0] + (deltaS * elength[m->edge(M)]);

			// Shrink triangle to avoid sampling edges
			triangle[ verts[vP] ] = points[vP] + (-deltaS * elength[m->edge(S)] * 0.001);
			triangle[ verts[vM] ] = points[vM] + (-deltaL * elength[m->edge(L)] * 0.001);
		}

		double varea = 0.5 * cross((virtualTri[1] - virtualTri[0]), (virtualTri[2] - virtualTri[0])).norm();
		
        // compute # samples in the current face
        int n_samples = (int) (0.5 * varea * samplePerAreaUnit);

        if(n_samples > 1)
        {
            int n_samples_per_edge = (int)((sqrt(1.0 + 8.0 * (Scalar)n_samples) + 5.0) / 2.0);

            Scalar segmentNum = n_samples_per_edge - 1 ;
            Scalar segmentLen = 1.0 / segmentNum;

            // face sampling
            for(int i = 1; i < (n_samples_per_edge - 1); i++)
			{
                for(int j = 1; j < (n_samples_per_edge - 1) - i; j++)
                {
                    Scalar uvw[] = {i*segmentLen, j*segmentLen, 1.0 - (i*segmentLen + j*segmentLen)};

                    // Get point from current barycentric coordinate
                    Vector3 p(0.0); for(int vi = 0; vi < 3; vi++) p += virtualTri[vi] * uvw[vi];

					Vec3d coord = barycentric(p, triangle[0], triangle[1], triangle[2]);
					if( !isValidBaryCoord (coord) ) continue;

                    samples.push_back( p );
                    samplesNormals.push_back( fnormals[f] );

					if(avgSpacing && j > 1 && samples.size() > 1)
						*avgSpacing = (samples.back() - samples[samples.size()-2]).norm();
                }
            }
        }
    }

    return samples;
}

QVector<Vector3> SimilarSampler::EdgeUniform(SurfaceMeshModel * m, int sampleNum, QVector<Vector3> & samplesNormals)
{
    SurfaceMeshHelper h( m );

    // First loop compute total edge lenght;
    Scalar edgeSum = 0;
    if(!m->has_edge_property<Scalar>(ELENGTH)) h.computeEdgeLengths();
    ScalarEdgeProperty elength = m->edge_property<Scalar>(ELENGTH);
    foreach(Edge e, m->edges())	edgeSum += elength[e];

    Scalar sampleLen = edgeSum / sampleNum;

	return EdgeUniformFixed(m,samplesNormals,sampleLen);
}

QVector<Vector3> SimilarSampler::EdgeUniformFixed( SurfaceMeshModel * m, QVector<Vector3> & samplesNormals, double sampleLen )
{ 
	QVector<Vector3> samples;

	SurfaceMeshHelper h( m );

	// Mesh points
	Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);

	// Face normals
	m->update_face_normals();
	Vector3FaceProperty fnormals = m->get_face_property<Vector3>(FNORMAL);

	if(!m->has_edge_property<Scalar>(ELENGTH)) SurfaceMeshHelper(m).computeEdgeLengths();
	ScalarEdgeProperty elength = m->edge_property<Scalar>(ELENGTH);
	
	Scalar rest = 0;

	foreach(Edge ei, m->edges()){
		Scalar len = elength[ei];
		Scalar samplePerEdge = floor((len+rest)/sampleLen);
		rest = (len+rest) - samplePerEdge * sampleLen;
		Scalar step = 1.0 / (samplePerEdge + 1);
		for(int i = 0; i < samplePerEdge; ++i)
		{
			Scalar alpha = step*(i+1);
			Scalar beta = 1.0 - step*(i+1);
			samples.push_back( (alpha * points[m->vertex(ei,0)]) + (beta * points[m->vertex(ei,1)]) );

			// Normal = average of adj faces
			{
				Vec3d normal(0);
				Face f1 = m->face(m->halfedge(ei,0)),f2 = m->face(m->halfedge(ei,1));
				if(f1.is_valid()) normal += fnormals[f1];
				if(f2.is_valid()) normal += fnormals[f1];
				if(f1.is_valid() && f2.is_valid()) normal /= 2.0;

				samplesNormals.push_back( normal );
			}
		}
	}

	return samples;
}

QVector<Vector3> SimilarSampler::All( SurfaceMeshModel * m, int sampleNum, QVector<Vector3> & samplesNormals )
{
	QVector<Vector3> samples;

	double sampleSpacing = 0;
	samples += SimilarSampler::FaceSamples( m, sampleNum, samplesNormals, &sampleSpacing );
	samples += SimilarSampler::EdgeUniformFixed( m, samplesNormals, sampleSpacing );
	samples += SimilarSampler::Vertices( m, samplesNormals );

	return samples;
}
