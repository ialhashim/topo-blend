#include "visiblity_resampler.h"
#include "Octree.h"
#include "StarlabDrawArea.h"

#include "../CustomDrawObjects.h"
#include "../TopoBlenderLib/Sampler.h"
#include "../TopoBlenderLib/Sampler.cpp"

// OpenMP
#include <omp.h>

#include "../NURBS/weld.h"

void traverseOctree( Octree & octree, CubeSoup & cs )
{
	if( octree.children.empty() && !octree.triangleData.empty() )
	{
		cs.addCube( octree.boundingBox.Center(), octree.boundingBox.yExtent * 2 );
		return;
	}

	foreach(Octree t, octree.children)
	{
		traverseOctree(t,cs);
	}
}

void visiblity_resampler::initParameters(RichParameterSet *pars)
{
    pars->addParam(new RichBool("saveFileXYZ",true,"Save points to XYZ file"));
	pars->addParam(new RichBool("addModel",false,"Add as layer"));
	pars->addParam(new RichBool("viz",true,"Visualize"));
	pars->addParam(new RichInt("randSamples",1e4,"Number of random samples"));
}

std::vector<Vec3d> randomSampleSphere( int numSamples, bool isRegular )
{
	double theta,rho,phi;
	double x,y,z;

	std::vector<Vec3d> samples;

	int iAQuantize = 200;
	int iBQuantize = 50;
	double fAQuantize = (double)(iAQuantize-1);
	double fBQuantize = (double)(iBQuantize-1);

	for (int i = 0; i < numSamples; i++)
	{
		// choose a theta & rho
		if (isRegular)
		{
			theta = ((double)(rand() % iAQuantize)) / fAQuantize;
			rho   = ((double)(rand() % iBQuantize)) / fBQuantize;
		}
		else
		{
			theta = ((double)rand()) / (double)RAND_MAX;
			rho   = ((double)rand()) / (double)RAND_MAX;
		}

		// theta and rho now between 0 and 1
		theta *= 2.0 * 3.14159265359;
		rho   *= 2.0;

		// convert rho to phi;
		phi   = rho * 3.14159265359f * 0.5f;

		// spherical to Cartesian
		x = sin(phi) * cos(theta);
		y = cos(phi);
		z = sin(phi) * sin(theta);

		samples.push_back(Vec3d(x,y,z));
	}

	return samples;
}

std::vector<Vec3d> uniformSampleSphere( int numSamples = 5 )
{
	double x,y,z;

	std::vector<Vec3d> samples;

	double d_theta = (M_PI) / numSamples;
	double d_psi = (M_PI) / numSamples;

	for(double theta = 0; theta <= M_PI; theta += d_theta)
	{
		for(double psi = 0; psi <= 2 * M_PI; psi += d_psi)
		{
			x = sin(theta) * cos(psi);
			y = sin(theta) * sin(psi);
			z = cos(theta);

			samples.push_back(Vec3d(x,y,z));
		}
	}

	return samples;
}

void visiblity_resampler::applyFilter(RichParameterSet *pars)
{		
	SurfaceMeshHelper h(mesh());
	Vector3VertexProperty points = mesh()->vertex_property<Vector3>(VPOINT);
	Vector3FaceProperty fnormals = h.computeFaceNormals();

	double surfaceOffset = 1e-6;

    Octree octree(mesh());

	std::vector<Vec3d> sphere = uniformSampleSphere( 5 );
	int rayCount = sphere.size();

	Sampler sampler(mesh());
	std::vector<SamplePoint> all_samples = sampler.getSamples( pars->getInt("randSamples") );

	int N = (int) all_samples.size();
	std::vector<bool> isUse(N,false);

	#pragma omp parallel for
	for(int i = 0; i < N; i++)
	{
		const SamplePoint & sp = all_samples[i];

		for(int r = 0; r < rayCount; r++)
		{
			const Vec3d & d = sphere[r];

			Ray ray( sp.pos + (d * surfaceOffset), d );
			
			if(octree.intersectRay(ray,0,true).empty())
			{
				isUse[i] = true;
				break;
			}
		}
	}

	document()->pushBusy();

	PointSoup * ps = NULL;
	SurfaceMesh::Model * m = NULL;

	QString newMeshName = QString("%1_sampled").arg(mesh()->name);

	if(pars->getBool("addModel")) m = new SurfaceMesh::Model( newMeshName+".obj", newMeshName );
	if(pars->getBool("viz")) ps = new PointSoup;

	QTextStream * out = NULL;
	QString filename = "";
	if(pars->getBool("saveFileXYZ")) filename = QFileDialog::getSaveFileName(NULL,"Save XYZ","./", "XYZ file (*.xyz)");
	QFile file(filename); 
	if(pars->getBool("saveFileXYZ")){
		file.open(QFile::WriteOnly | QFile::Text);
		out = new QTextStream(&file);
	}

	std::vector<SamplePoint> used_samples;
	std::vector<Vec3d> used_samples_points;

	for(int i = 0; i < N; i++){
		if(isUse[i]) 
		{
			used_samples.push_back(all_samples[i]);
			used_samples_points.push_back(all_samples[i].pos);
		}
	}

	std::vector<size_t> corner_xrefs;
	weld(used_samples_points, corner_xrefs, std::hash_Vec3d(), std::equal_to<Vec3d>());


	for(int j = 0; j < (int)used_samples.size(); j++)
	{
		int i = corner_xrefs[j];

		const SamplePoint& sp = used_samples[i];

		if(pars->getBool("viz")) ps->addPointNormal( Vec3d(sp.pos), fnormals[Face(sp.findex)] );
		if(pars->getBool("addModel")) m->add_vertex( sp.pos );

		if(pars->getBool("saveFileXYZ"))
		{
			Vec3d p = sp.pos;
			Vec3d n = fnormals[Face(sp.findex)];
			(*out) << QString("%1 %2 %3 %4 %5 %6\n").arg(p[0]).arg(p[1]).arg(p[2]).arg(n[0]).arg(n[1]).arg(n[2]);
		}

	}

	file.close();

	if(pars->getBool("addModel"))
	{
		m->updateBoundingBox();
		document()->addModel(m);
	}

	drawArea()->deleteAllRenderObjects();
	if(pars->getBool("viz")) 
	{
		drawArea()->addRenderObject(ps);
	}

	document()->popBusy();

	drawArea()->updateGL();
}

Q_EXPORT_PLUGIN(visiblity_resampler)
