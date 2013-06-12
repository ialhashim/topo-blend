#pragma once
#include "Sampler.h"
#include "NanoKdTree.h"

class SpherePackSampling{

public:
    static std::vector<Vector3d> sample(SurfaceMesh::Model * m, int randomSampleCount, double r)
    {
        std::vector<Vector3d> gridPoints = std::vector<Vector3d>();
        return sample(m,randomSampleCount,r,gridPoints);
    }

    static std::vector<Vector3d> sample(SurfaceMesh::Model * m, int randomSampleCount, double r, 
        std::vector<Vector3d> & gridPoints, int density = 1)
	{
        std::vector<Vector3d> samples, centers, rndSamples;

        // Centers of packed spheres
        centers = spheres(r, m->bbox().min(), m->bbox().max(), density);

		// Get a lot of random samples
		foreach(SamplePoint sp, Sampler(m).getSamples(randomSampleCount)){
			rndSamples.push_back(sp.pos);
		}

		// Initialize KD-tree
		NanoKdTree tree;
		foreach(Vector3d p, rndSamples) tree.addPoint(p);
		tree.build();

        foreach(Vector3d center, centers)
        {
			// Collect neighbors
			KDResults matches;
			int n = tree.ball_search( center, r, matches );

            if(n < 1) continue;

			// Record center
			Vector3d centerGroup (0,0,0);
			foreach(KDResultPair i, matches) 
				centerGroup += rndSamples[i.first];
			centerGroup /= matches.size();
			samples.push_back(centerGroup);

			gridPoints.push_back(center);
		}


		return samples;
	}

	/* Hexagonal close packing of spheres (HCP lattice) */
    static std::vector<Vector3d> spheres(double r, Vector3d bbmin, Vector3d bbmax, int density = 1)
	{
        std::vector<Vector3d> samples;

        std::vector< std::vector<Vector3d> > grid;

		double d = r * 2;

		double width = bbmax.x() - bbmin.x();
		double length = bbmax.y() - bbmin.y();
		double height = bbmax.z() - bbmin.z();

		Vector3d corner(bbmin.x()-r, bbmin.y()-r, bbmin.z());

		int dx = (width / r) + 1;
		int dy = (length / r) + 1;
		int dz = (height / r) + 1;

		Vector3d delta(r, sqrt(3.0) * r , 0);
		Vector3d center(0,0,0);

		grid.push_back(std::vector<Vector3d>());

		for(int x = 0; x < dx; x++)
			grid[0].push_back(corner + Vector3d(x * d, 0, 0));

		for(int y = 0; y < dy; y++){
			std::vector<Vector3d> row = grid.back();
			
			for(int x = 0; x < (int)row.size(); x++)	row[x] += delta;

			delta.x() = -delta.x();

			grid.push_back(row);
		}

		Vector3d omega(r, r, sqrt(6.0) * (2.0/3.0) * r);

		for(int z = 0; z < dz; z += density){
			for(int y = 0; y < dy; y += density)
				for(int x = 0; x < dx; x += density)
                    samples.push_back(grid[y][x] + center);
				
			omega.x() *= -1;
			omega.y() *= -1;

			center += omega;
		}

		return samples;
	}

	static std::vector<double> toKDPoint(const SurfaceMesh::Point & from){
		std::vector<double> p(3, 0.0);
		p[0] = from.x(); p[1] = from.y(); p[2] = from.z();
		return p;
	}
};
