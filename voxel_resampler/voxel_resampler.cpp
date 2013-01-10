#include "voxel_resampler.h"

#include "DynamicVoxel.h"
#include "Voxeler.h"

#include <QStack>
#include <QSet>

void voxel_resampler::initParameters(RichParameterSet *pars)
{
	pars->addParam(new RichFloat("voxel_scale", 0.1f, "Voxel scale", "Voxel scale in % of maximum bounding extent"));
	pars->addParam(new RichBool("keep_inside", false, "Keep inner sheels", ""));
	pars->addParam(new RichFloat("mcf_smoothing", 0.0f, "MCF Smoothing", ""));
	pars->addParam(new RichInt("laplacian_smoothing", 3, "Laplacian Smoothing", ""));
}

void voxel_resampler::applyFilter(RichParameterSet *pars)
{
	// Voxel size
	double vox_scale = pars->getFloat("voxel_scale");
	double voxel_size = vox_scale * mesh()->bbox().size().length();

	// Voxelize the mesh
	VoxelerLibrary::Voxeler voxeler(mesh(), voxel_size, true);
	//std::vector<VoxelerLibrary::Voxel> voxels = voxeler.fillInside();
	std::vector<VoxelerLibrary::Voxel> voxels = voxeler.voxels;

	// Add voxels to a Dynamic Voxel object
	DynamicVoxelLib::DynamicVoxel vox(voxel_size);
	vox.begin();
	foreach(VoxelerLibrary::Voxel v, voxels) vox.setVoxel(v.x, v.y, v.z);
	vox.end();

	// Build mesh
	SurfaceMeshModel * m = new SurfaceMeshModel("voxelized.obj", "voxelized");

	vox.buildMesh( m );
	m->triangulate();

	/// Remove inner shells
	bool keep_inner = pars->getBool("keep_inside");
	
	if(!keep_inner)
	{
		// Find new mesh segments
		SurfaceMeshModel * outerMesh = new SurfaceMeshModel("voxelized_outer.obj", "voxelized_outer_shell");
		Vector3VertexProperty points = m->vertex_property<Vector3>(VPOINT);

		QVector< QSet<int> > segments;
		SurfaceMeshModel::Face_property<bool> fvisited = m->add_face_property<bool>("f:visited", false);
		int visitedCount = 0;

		while(visitedCount < (int)m->n_faces()){
			QStack<SurfaceMeshModel::Face> tovisit;

			// Seed face
			foreach(SurfaceMeshModel::Face f, m->faces()){
				if(!fvisited[f]){
					tovisit.push(f);
					break;
				}
			}

			segments.push_back( QSet<int>() );

			while( !tovisit.isEmpty() )	{
				SurfaceMeshModel::Face f = tovisit.pop();
				SurfaceMeshModel::Halfedge_around_face_circulator adjE(m, f), eend = adjE;
				fvisited[f] = true;
				segments.back().insert(f.idx());

				do{ 
					Face adjF = m->face( m->opposite_halfedge(adjE) );

					if( !fvisited[adjF] ){
						tovisit.push(adjF);
						fvisited[adjF] = true;
						segments.back().insert(adjF.idx());
					}
				} while(++adjE != eend);
			}

			visitedCount += segments.back().size();
		}

		// Sorted by size
		QMap< int, QSet<int> > segmentMap;
		foreach(QSet<int> segment, segments) segmentMap[segment.size()] = segment;

		// Assumption: keep every other
		QVector< QSet<int> > allSegments = segmentMap.values().toVector();

		// Collect set of used vertices
		QSet<int> vset, fset;
		QMap<int,int> vmap;

		for (int si = 0; si < (int)allSegments.size(); si++)
		{
			// Even indices starting from zero
			if(si % 2 == 0) continue;

			foreach( int fidx, allSegments[si] )
			{
				Face face(fidx);
				Surface_mesh::Vertex_around_face_circulator vit = m->vertices(face),vend=vit;
				do{ vset.insert( Vertex(vit).idx() ); } while(++vit != vend);
				fset.insert(fidx);
			}
		}

		// Remap vertex indices and add to new mesh
		foreach(int vi, vset)
		{
			vmap[vi] = vmap.size();
			outerMesh->add_vertex( points[Vertex(vi)] );
		}

		// Add faces to new mesh
		foreach(int fi, fset)
		{
			std::vector<Vertex> verts;
			Surface_mesh::Vertex_around_face_circulator vit = m->vertices(Face(fi)),vend=vit;
			do{ verts.push_back( Vertex( vmap[ Vertex(vit).idx() ] ) ); } while(++vit != vend);
			outerMesh->add_face(verts);
		}

		m = outerMesh;
	}

	if(pars->getFloat("mcf_smoothing") > 0.0)
		DynamicVoxelLib::DynamicVoxel::MeanCurvatureFlow( m, voxel_size * pars->getFloat("mcf_smoothing") );
	
	if(pars->getInt("laplacian_smoothing") > 0)
	{
		for(int i = 0; i < pars->getInt("laplacian_smoothing"); i++)
			vox.LaplacianSmoothing( m );
	}

	document()->addModel(m);
}

Q_EXPORT_PLUGIN(voxel_resampler)
