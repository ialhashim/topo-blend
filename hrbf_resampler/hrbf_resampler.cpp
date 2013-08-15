#include "hrbf_resampler.h"

// Sampling and welding
#include "../TopoBlenderLib/SimilarSampling.cpp"
#include "../NURBS/weld.h"

// HRBF
#include "hrbf/hrbf_phi_funcs.h"
#include "hrbf/hrbf_core.h"
typedef HRBF_fit<double, 3, Rbf_pow3<double> > HRBF;

// Marching cubes
#include "mc/MarchingCubes.h"

#include "NanoKdTree.h"

void hrbf_resampler::initParameters(RichParameterSet *pars)
{
    // Sampling parameters
    pars->addParam(new RichInt("numSamples", 10, "Number of samples"));

    // Marching Cubes parameters
    pars->addParam(new RichFloat("cellScale", 0.1f, "Cell scale", "Cell scale in % of maximum bounding extent"));
}

GRIDCELL evalCell(Vector3 p, Scalar cellSize, HRBF & fit){
	GRIDCELL cell;
	Vector3 deltaX(cellSize,0,0), deltaY(0,cellSize,0), deltaZ(0,0,cellSize);

	cell.p[0] = p;						cell.val[0] = fit.eval(cell.p[0]);
	cell.p[1] = p + deltaX;				cell.val[1] = fit.eval(cell.p[1]);
	cell.p[2] = p + deltaX + deltaY;	cell.val[2] = fit.eval(cell.p[2]);
	cell.p[3] = p + deltaY;				cell.val[3] = fit.eval(cell.p[3]);

	p += deltaZ;

	cell.p[4] = p;						cell.val[4] = fit.eval(cell.p[4]);
	cell.p[5] = p + deltaX;				cell.val[5] = fit.eval(cell.p[5]);
	cell.p[6] = p + deltaX + deltaY;	cell.val[6] = fit.eval(cell.p[6]);
	cell.p[7] = p + deltaY;				cell.val[7] = fit.eval(cell.p[7]);

	return cell;
}

void hrbf_resampler::applyFilter(RichParameterSet *pars)
{
    // Sample mesh
    QVector<Vector3> m_samples, m_normals;
	m_samples = SimilarSampler::All(mesh(), pars->getInt("numSamples"), m_normals);  // Similar sampling
	std::vector<Vector3> points = m_samples.toStdVector(), normals = m_normals.toStdVector(); // convert to std::vector

	foreach(Vector3 p, m_samples) drawArea()->drawPoint(p);

	// Compute grid parameters
	mesh()->updateBoundingBox();
	Vector3 ext = mesh()->bbox().diagonal();
	double cellSize = ext.norm() * pars->getFloat("cellScale");
	int countX = qMax(1.0, ext.x() / cellSize), countY = qMax(1.0, ext.y() / cellSize), countZ = qMax(1.0, ext.z() / cellSize);
	Vector3 startCorner = mesh()->bbox().min();

    // Construct HRBF fit of samples
	HRBF fit; 
	fit.hermite_fit(points, normals);

	/// Extract iso-surface via Marching Cubes

	// Compute active grid cells by looking at samples
	std::vector<Vector3> vertices;
	std::vector<Vector3i> tris;
	int voffset = 0;

	NanoKdTree kdtree;
	for(int x = -2; x <= countX + 1; x++){
		for(int y = -2; y <= countY + 1; y++){
			for(int z = -2; z <= countZ + 1; z++){
				GRIDCELL cell = evalCell(startCorner + (Vector3(x, y, z) * cellSize), cellSize, fit);
				
				TriMeshChunk chunk;
				if( Polygonise(cell, chunk) )
				{
					foreach(Vector3 p, chunk.vertices) { vertices.push_back(p); kdtree.addPoint(p); }
					foreach(TriMeshFace f, chunk.faces)	tris.push_back( Vector3i(f[2] + voffset, f[1] + voffset, f[0] + voffset) );
					voffset += chunk.vertices.size();
				}
			}
		}
	}

	mesh()->isVisible = false;
	mesh()->clear();

	// Vertices should be unique
	double threshold = 1e-10;
	kdtree.build();

	std::vector<KDResults> groups;
	for(int i = 0; i < (int)vertices.size(); i++){
		Vector3 p = vertices[i];

		KDResults matches;
		kdtree.ball_search(p, threshold, matches);

		double diffs = 0;
		foreach(KDResultPair r, matches) diffs += r.second;
		if( matches.size() > 1 && diffs > 0){
			groups.push_back(matches);
		}
	}

	// Snap matches to first element
	foreach(KDResults matches, groups){
		Vector3 snapPoint = vertices[ matches.front().first ];
		for(int i = 1; i < (int)matches.size(); i++){
			vertices[matches[i].first] = snapPoint;
		}
	}

	std::vector<size_t> xrefs;
	weld(vertices, xrefs, std::hash_Vector3d(), std::equal_to<Vector3d>());

	// Add re-sampled mesh
	foreach(Vector3 p, vertices) mesh()->add_vertex(p);
	foreach(Vector3i f, tris){
		int v1 = xrefs[f[0]];
		int v2 = xrefs[f[1]];
		int v3 = xrefs[f[2]];

		if(v1 != v2 && v2 != v3 && v1 != v3)
			mesh()->add_triangle(Vertex(v1),Vertex(v2),Vertex(v3));
	}

	mesh()->updateBoundingBox();
	mesh()->update_face_normals();
	mesh()->update_vertex_normals();
	mesh()->isVisible = true;

}

Q_EXPORT_PLUGIN(hrbf_resampler)
