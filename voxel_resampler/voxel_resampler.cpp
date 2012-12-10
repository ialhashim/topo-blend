#include "voxel_resampler.h"

#include "DynamicVoxel.h"
#include "Voxeler.h"

void voxel_resampler::initParameters(RichParameterSet *pars)
{
	pars->addParam(new RichFloat("voxel_scale", 0.1f, "Voxel scale", "Voxel scale in % of maximum bounding extent"));
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

	vox.MeanCurvatureFlow( m, 0.1 * voxel_size );
	//vox.LaplacianSmoothing( m );

	document()->addModel(m);
}

Q_EXPORT_PLUGIN(voxel_resampler)
