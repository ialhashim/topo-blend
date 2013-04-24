#pragma once

#include "SurfaceMeshHelper.h"
using namespace SurfaceMesh;

struct SimilarSampler{

	// Similar Triangles sampling. [Meshlab]
	// Skip vertex and edges
    static QVector<Vector3> FaceSamples(SurfaceMeshModel * m, int sampleNum, QVector<Vector3> & samplesNormals);

    static QVector<Vector3> FaceSamples(SurfaceMeshModel * m, int sampleNum = 100000)
	{
        QVector<Vector3> samplesNormals;
		return FaceSamples(m, sampleNum, samplesNormals);
	}

    static QVector<Vector3> EdgeUniform(SurfaceMeshModel * m, int sampleNum, QVector<Vector3> & samplesNormals);

    static QVector<Vector3> EdgeUniform(SurfaceMeshModel * m, int sampleNum = 100000)
	{
        QVector<Vector3> samplesNormals;
		return EdgeUniform(m, sampleNum, samplesNormals);
	}

    static QVector<Vector3> Vertices(SurfaceMeshModel * m, QVector<Vector3> & samplesNormals)
	{	
        QVector<Vector3> samples;

		// Mesh points
        Vector3VertexProperty points = m->vertex_property<Vector3>(VPOINT);

		// Normals
		m->update_face_normals();
		m->update_vertex_normals();
        Vector3VertexProperty normals = m->vertex_property<Vector3>(VNORMAL);

		foreach(Vertex v, m->vertices()) 
		{
			samples.push_back(points[v]);
			samplesNormals.push_back(normals[v]);
		}

		return samples;
	}

    static QVector<Vector3> Vertices(SurfaceMeshModel * m)
	{
        QVector<Vector3> samplesNormals;
		return Vertices(m, samplesNormals);
	}

    static QVector<Vector3> All(SurfaceMeshModel * m, int sampleNum, QVector<Vector3> & samplesNormals){
        QVector<Vector3> samples;

		samples += SimilarSampler::FaceSamples( m, sampleNum, samplesNormals );
		samples += SimilarSampler::EdgeUniform( m, sampleNum / 4, samplesNormals );
		samples += SimilarSampler::Vertices( m, samplesNormals );

		return samples;
	}

    static QVector<Vector3> All(SurfaceMeshModel * m, int sampleNum = 100000){
        QVector<Vector3> samplesNormals;
		return All(m,sampleNum,samplesNormals);
	}
};
