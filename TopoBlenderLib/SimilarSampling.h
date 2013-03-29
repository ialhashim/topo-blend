#pragma once

#include "SurfaceMeshHelper.h"

struct SimilarSampler{

	// Similar Triangles sampling. [Meshlab]
	// Skip vertex and edges
	// Sample per edges includes vertexes, so here we should expect n_samples_per_edge >=4 
	static QVector<Vector3> FaceSamples(SurfaceMeshModel * m, int sampleNum, QVector<Vector3> & samplesNormals)
	{	
		QVector<Vector3> samples;

		// Compute face areas
		Scalar area = 0;
		SurfaceMeshHelper h( m );
		h.computeFaceAreas();
		ScalarFaceProperty farea = m->face_property<Scalar>(FAREA);
		Vector3FaceProperty fnormals = h.computeFaceNormals();
		foreach(Face f, m->faces()) area += farea[f];
		
		Scalar samplePerAreaUnit = sampleNum / area;

		// Mesh points
		Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);

		// Similar Triangles sampling.
		int n_samples_per_edge;
		Scalar n_samples_decimal = 0.0;

		foreach(SurfaceMeshModel::Face f, m->faces())
		{
			// compute # samples in the current face.
			n_samples_decimal += 0.5 * farea[f] * samplePerAreaUnit;
			int n_samples = (int) n_samples_decimal;
			if(n_samples > 0)
			{
				n_samples_per_edge = (int)((sqrt(1.0 + 8.0 * (Scalar)n_samples) + 5.0) / 2.0); // original for non dual case
				
				n_samples = 0;
				int i, j;
				Scalar segmentNum = n_samples_per_edge - 1 ;
				Scalar segmentLen = 1.0 / segmentNum;

				// face sampling
				for(i=1; i < n_samples_per_edge - 1; i++){
					for(j=1; j < n_samples_per_edge - 1 - i; j++)
					{
						Scalar uvw[] = {i*segmentLen, j*segmentLen, 1.0 - (i*segmentLen + j*segmentLen)};
						Vector3 p(0.0);

						// Get point from barycentric coords
						int vi = 0;
						Surface_mesh::Vertex_around_face_circulator vit = m->vertices(f),vend=vit;
						do{ p += points[vit] * uvw[vi++]; } while(++vit != vend);

						samples.push_back( p );
						samplesNormals.push_back( fnormals[f] );
						n_samples++;
					}
				}
			}
			n_samples_decimal -= (Scalar) n_samples;
		}

		return samples;
	}

	static QVector<Vector3> FaceSamples(SurfaceMeshModel * m, int sampleNum = 100000)
	{
		QVector<Vector3> samplesNormals;
		return FaceSamples(m, sampleNum, samplesNormals);
	}

	static QVector<Vector3> EdgeUniform(SurfaceMeshModel * m, int sampleNum, QVector<Vector3> & samplesNormals)
	{	
		QVector<Vector3> samples;

		SurfaceMeshHelper h( m );

		// Mesh points
		Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);

		// Face normals
		Vector3FaceProperty fnormals = h.computeFaceNormals();

		// First loop compute total edge lenght;
		Scalar edgeSum = 0;
		if(!m->has_edge_property<Scalar>(ELENGTH)) h.computeEdgeLengths();
		ScalarEdgeProperty elength = m->edge_property<Scalar>(ELENGTH);
		foreach(Edge e, m->edges())	edgeSum += elength[e];

		Scalar sampleLen = edgeSum/sampleNum;
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
