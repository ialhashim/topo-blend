#include "voxel_resampler.h"

#include "DynamicVoxel.h"
#include "Voxeler.h"

#include <QStack>
#include <QSet>

void voxel_resampler::initParameters(RichParameterSet *pars)
{
	pars->addParam(new RichBool("apply_original", true, "Apply to original", ""));
	pars->addParam(new RichFloat("voxel_scale", 0.1f, "Voxel scale", "Voxel scale in % of maximum bounding extent"));
	pars->addParam(new RichBool("keep_inside", false, "Keep inner sheels", ""));
	pars->addParam(new RichFloat("mcf_smoothing", 0.0f, "MCF Smoothing", ""));
	pars->addParam(new RichInt("laplacian_smoothing", 2, "Laplacian Smoothing", ""));
}

void voxel_resampler::applyFilter(RichParameterSet *pars)
{
	// Voxel size
	double vox_scale = pars->getFloat("voxel_scale");
	double voxel_size = vox_scale * mesh()->bbox().diagonal().norm();

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
	SurfaceMesh::Model m (mesh()->path, mesh()->name);
	SurfaceMeshModel * outMesh = &m; // assume we use all output of voxelization step

	vox.buildMesh( &m );
	m.triangulate();

	bool apply_original = pars->getBool("apply_original");

	if( apply_original )
	{
		// Remove original geometry
		outMesh = mesh();
	}
	else
	{
		outMesh = new SurfaceMeshModel(m.path, m.name);
		document()->addModel( outMesh );
	}

	/// Remove inner shells if requested
	if( !pars->getBool("keep_inside") )
	{
		// Find new mesh segments
		Vector3VertexProperty points = m.vertex_property<Vector3>(VPOINT);

		QVector< QSet<int> > segments;
		SurfaceMesh::Model::Face_property<bool> fvisited = m.add_face_property<bool>("f:visited", false);
		int visitedCount = 0;

		while(visitedCount < (int)m.n_faces()){
			QStack<SurfaceMesh::Model::Face> tovisit;

			// Seed face
			foreach(SurfaceMesh::Model::Face f, m.faces()){
				if(!fvisited[f]){
					tovisit.push(f);
					break;
				}
			}

			segments.push_back( QSet<int>() );

			while( !tovisit.isEmpty() )	{
				SurfaceMesh::Model::Face f = tovisit.pop();
				SurfaceMesh::Model::Halfedge_around_face_circulator adjE(&m, f), eend = adjE;
				fvisited[f] = true;
				segments.back().insert(f.idx());

				do{ 
					Face adjF = m.face( m.opposite_halfedge(adjE) );

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

		outMesh->isVisible = false;
		outMesh->clear();

		// One segment case: Skip extraction step
		if(allSegments.size() == 1){
			foreach(Vertex v, m.vertices()) outMesh->add_vertex( points[v] );
			foreach(Face f, m.faces()){
				std::vector<Vertex> verts;
				Surface_mesh::Vertex_around_face_circulator vit = m.vertices(f),vend=vit;
				do{ verts.push_back( vit ); } while(++vit != vend);
				outMesh->add_face(verts);
			}
		}
		else
		{
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
					Surface_mesh::Vertex_around_face_circulator vit = m.vertices(face),vend=vit;
					do{ vset.insert( Vertex(vit).idx() ); } while(++vit != vend);
					fset.insert(fidx);
				}
			}

			// Remap vertex indices and add to new mesh
			foreach(int vi, vset)
			{
				vmap[vi] = vmap.size();
				outMesh->add_vertex( points[Vertex(vi)] );
			}

			// Add faces to new mesh
			foreach(int fi, fset)
			{
				std::vector<Vertex> verts;
				Surface_mesh::Vertex_around_face_circulator vit = m.vertices(Face(fi)),vend=vit;
				do{ verts.push_back( Vertex( vmap[ Vertex(vit).idx() ] ) ); } while(++vit != vend);
				outMesh->add_face(verts);
			}
		}
	}

	// Post processing:
	if(pars->getFloat("mcf_smoothing") > 0.0) DynamicVoxelLib::DynamicVoxel::MeanCurvatureFlow( outMesh, voxel_size * pars->getFloat("mcf_smoothing") );
	if(pars->getInt("laplacian_smoothing") > 0){
		for(int i = 0; i < pars->getInt("laplacian_smoothing"); i++)
			vox.LaplacianSmoothing( outMesh );
	}

	// Clean up
	outMesh->updateBoundingBox();
	outMesh->update_face_normals();
	outMesh->update_vertex_normals();
	outMesh->isVisible = true;
}

Q_EXPORT_PLUGIN(voxel_resampler)
