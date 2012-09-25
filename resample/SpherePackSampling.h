#pragma once
#include "Sampler.h"
#include "KDTree.h"
using namespace kdtree;

class SpherePackSampling{

public:
    static std::vector<Vec3d> sample(SurfaceMeshModel * m, int randomSampleCount, double r, 
		std::vector<Vec3d> & gridPoints = std::vector<Vec3d>(), int density = 1)
	{
        std::vector<Vec3d> samples, centers, rndSamples;

        // Centers of packed spheres
        centers = spheres(r, m->bbox().minimum(), m->bbox().maximum(), density);

		// Get a lot of random samples
		foreach(SamplePoint sp, Sampler(m).getSamples(randomSampleCount)){
			rndSamples.push_back(sp.pos);
		}

		// Add into a KD-tree
        std::vector< vector<double> > points;
        foreach(Vec3d p, rndSamples)
            points.push_back( toKDPoint(p) );
        KDTree tree(points);

        foreach(Vec3d center, centers)
        {
			// Collect neighbors
            std::vector<int> idxs;
            std::vector<double> dists;

            tree.ball_query(toKDPoint(center), r, idxs, dists);

            if(!idxs.size()) continue;

			// Record center
			Vec3d centerGroup (0,0,0);
            foreach(int i, idxs) centerGroup += Vec3d(points[i][0],points[i][1],points[i][2]);
            centerGroup /= idxs.size();
            samples.push_back(centerGroup);

			gridPoints.push_back(center);
		}

		return samples;
	}

	/* Hexagonal close packing of spheres (HCP lattice) */
    static std::vector<Vec3d> spheres(double r, Vec3d bbmin, Vec3d bbmax, int density = 1)
	{
        std::vector<Vec3d> samples;

        std::vector< std::vector<Vec3d> > grid;

		double d = r * 2;

		double width = bbmax.x() - bbmin.x();
		double length = bbmax.y() - bbmin.y();
		double height = bbmax.z() - bbmin.z();

		Vec3d corner(bbmin.x()-r, bbmin.y()-r, bbmin.z());

		int dx = (width / r) + 1;
		int dy = (length / r) + 1;
		int dz = (height / r) + 1;

		Vec3d delta(r, sqrt(3.0) * r , 0);
		Vec3d center(0,0,0);

		grid.push_back(std::vector<Vec3d>());

		for(int x = 0; x < dx; x++)
			grid[0].push_back(corner + Vec3d(x * d, 0, 0));

		for(int y = 0; y < dy; y++){
			std::vector<Vec3d> row = grid.back();
			
			for(int x = 0; x < (int)row.size(); x++)	row[x] += delta;

			delta.x() = -delta.x();

			grid.push_back(row);
		}

		Vec3d omega(r, r, sqrt(6.0) * (2.0/3.0) * r);

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

	static std::vector<double> toKDPoint(const SurfaceMeshTypes::Point & from){
		std::vector<double> p(3, 0.0);
		p[0] = from.x(); p[1] = from.y(); p[2] = from.z();
		return p;
	}
};
