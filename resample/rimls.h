#pragma once

// Robust implicit moving least squares (RIMLS) surfaces
// Paper: Feature Preserving Point Set Surfaces based on Non-Linear Kernel Regression
//		  A. Cengiz Öztireli, Gaël Guennebaud, Markus H. Gross. EG 2009
// Code: adapted from VCGLib

#include "SurfaceMeshHelper.h"
#include "NanoKdTree.h"

#include <Eigen/Core>
using namespace Eigen;

#ifndef V2E
#define V2E(vec) (Eigen::Vector3d(vec[0], vec[1], vec[2]))
#define E2V(vec) (Vec3d(vec[0], vec[1], vec[2]))
#endif

class RmlsSurface
{
	typedef Matrix3d MatrixType;

	enum {
		MLS_OK,	MLS_TOO_FAR, MLS_TOO_MANY_ITERS, MLS_NOT_SUPPORTED,
		MLS_DERIVATIVE_ACCURATE, MLS_DERIVATIVE_APPROX, MLS_DERIVATIVE_FINITEDIFF
	};

	Vector3VertexProperty points, normals;
	ScalarVertexProperty radii;

public:
	RmlsSurface(SurfaceMeshModel * mMesh) : mesh(mMesh)
	{
		mCachedQueryPointIsOK = false;
		mBallTree = 0;

		points = mesh->vertex_property<Vector3>(VPOINT);
		normals = mesh->vertex_property<Vector3>(VNORMAL);
		radii = mesh->vertex_property<Scalar>("v:radii", 0.0);

		mAABB = mesh->bbox();

		// Add original mesh points to KD-tree
		mBallTree = new NanoKdTree;
		foreach(Vertex v, mesh->vertices()) mBallTree->addPoint( points[v] );
		mBallTree->build();

		// compute radii using a basic mesh-less density estimator
		computeVertexRaddi();

		mFilterScale = 4.0;
		mMaxNofProjectionIterations = 20;
		mProjectionAccuracy = (Scalar)1e-4;

		mGradientHint = MLS_DERIVATIVE_ACCURATE;
		mHessianHint = MLS_DERIVATIVE_ACCURATE;

		mDomainMinNofNeighbors = 4;
		mDomainRadiusScale = 2.0;
		mDomainNormalScale = 1.0;

		mSigmaR = 0;
		mSigmaN = Scalar(0.8);
		mRefittingThreshold = Scalar(1e-3);
		mMinRefittingIters = 1;
		mMaxRefittingIters = 3;
	}

	~RmlsSurface()
	{
		delete mBallTree;
	}

	static const Scalar InvalidValue() { return Scalar(12345679810.11121314151617); }

	/** \returns the value of the reconstructed scalar field at point \a x */
	Scalar potential(const Vector3& x, int* errorMask = 0)
	{
		if ((!mCachedQueryPointIsOK) || mCachedQueryPoint!=x)
		{
			if (!computePotentialAndGradient(x))
			{
				if (errorMask)
					*errorMask = MLS_TOO_FAR;
				return InvalidValue();
			}
		}

		return mCachedPotential;
	}

	/** \returns the gradient of the reconstructed scalar field at point \a x
	*
	* The method used to compute the gradient can be controlled with setGradientHint().
	*/
	Vector3 gradient(const Vector3& x, int* errorMask = 0)
	{
		if ((!mCachedQueryPointIsOK) || mCachedQueryPoint!=x)
		{
			if (!computePotentialAndGradient(x))
			{
				if (errorMask)
					*errorMask = MLS_TOO_FAR;
				return Vector3(0,0,0);
			}
		}

		return mCachedGradient;
	}

	/** \returns the hessian matrix of the reconstructed scalar field at point \a x
	*
	* The method used to compute the hessian matrix can be controlled with setHessianHint().
	*/
	MatrixType hessian(const Vector3& x, int* errorMask = 0)
	{
		if ((!mCachedQueryPointIsOK) || mCachedQueryPoint!=x){
			if (!computePotentialAndGradient(x)){
				if (errorMask)
					*errorMask = MLS_TOO_FAR;
				return MatrixType();
			}
		}

		MatrixType hessian;
		mlsHessian(x, hessian);
		return hessian;
	}


	/** \returns the projection of point x onto the MLS surface, and optionally returns the normal in \a pNormal */
    Vector3 project(const Vector3& x)
    {
        Vector3 pNormal(0);
        return project(x, pNormal);
    }

    Vector3 project(const Vector3& x, Vector3 & pNormal, int* errorMask = 0)
	{
		int iterationCount = 0;
		Vector3 position = x;
		Vector3 normal;
		Scalar delta;
		Scalar epsilon = mAveragePointSpacing * mProjectionAccuracy;
		do {
			if (!computePotentialAndGradient(position))
			{
				if (errorMask) *errorMask = MLS_TOO_FAR;
				return x;
			}

			normal = mCachedGradient;
			normal.normalize();
			delta = mCachedPotential;
			position = position - normal*delta;
		} while ( abs(delta) > epsilon && ++iterationCount < mMaxNofProjectionIterations);

		if (iterationCount >= mMaxNofProjectionIterations && errorMask)
			*errorMask = MLS_TOO_MANY_ITERS;

		pNormal = normal;

		return position;
	}


	/** \returns whether \a x is inside the restricted surface definition domain */
	bool isInDomain(const Vector3& x)
	{
		if ((!mCachedQueryPointIsOK) || mCachedQueryPoint!=x){
			computeNeighborhood(x, false);
		}
		int nb = mNeighborhood.size();
		if (nb < mDomainMinNofNeighbors)
			return false;

		int i = 0;
		bool out = true;
		bool hasNormal = true;
		if ((mDomainNormalScale == 1.0) || (!hasNormal)){
			while (out && i<nb){
				int id = mNeighborhood[i].first;
				Scalar rs2 = radii[Vertex(id)] * mDomainRadiusScale;
				rs2 = rs2*rs2;
				out = pow(mNeighborhood[i].second, 2) > rs2;
				++i;
			}
		}
		else
		{
			Scalar s = 1./(mDomainNormalScale*mDomainNormalScale) - 1.f;
			while (out && i<nb){
				int id = mNeighborhood[i].first;
				Vertex v(id);
				Scalar rs2 = radii[v] * mDomainRadiusScale;
				rs2 = rs2*rs2;
				Scalar dn = dot(normals[v], x - points[v]);
				out = (pow(mNeighborhood[i].second, 2) + s*dn*dn) > rs2;
				++i;
			}
		}
		return !out;
	}

	/** \returns the mean curvature from the gradient vector and Hessian matrix.
	*/
	Scalar meanCurvature(const Vector3& gradient, const MatrixType& hessian) const
	{
		Scalar gl = gradient.norm();
		Vector3d hg = hessian * V2E(gradient);
		return ( gl*gl*hessian.trace() - dot(gradient, E2V(hg)) ) / (2.*gl*gl*gl);
	}

	/** set the scale of the spatial filter */
	void setFilterScale(Scalar v)
	{
		mFilterScale = v;
		mCachedQueryPointIsOK = false;
	}

	/** set the maximum number of iterations during the projection */
	void setMaxProjectionIters(int n)
	{
		mMaxNofProjectionIterations = n;
		mCachedQueryPointIsOK = false;
	}

	/** set the threshold factor to detect convergence of the iterations */
	void setProjectionAccuracy(Scalar v)
	{
		mProjectionAccuracy = v;
		mCachedQueryPointIsOK = false;
	}

	/** set a hint on how to compute the gradient
	*
	* Possible values are MLS_DERIVATIVE_ACCURATE, MLS_DERIVATIVE_APPROX, MLS_DERIVATIVE_FINITEDIFF
	*/
	void setGradientHint(int h)
	{
		mGradientHint = h;
		mCachedQueryPointIsOK = false;
	}

	/** set a hint on how to compute the hessian matrix
	*
	* Possible values are MLS_DERIVATIVE_ACCURATE, MLS_DERIVATIVE_APPROX, MLS_DERIVATIVE_FINITEDIFF
	*/
	void setHessianHint(int h)
	{
		mHessianHint = h;
		mCachedQueryPointIsOK = false;
	}

	const Eigen::AlignedBox3d& boundingBox() const { return mAABB; }

	void computeVertexRaddi(const int nbNeighbors = 16)
	{
		mAveragePointSpacing = 0;

		foreach(Vertex v, mesh->vertices())
		{
			KDResults matches;
			mBallTree->k_closest(points[v], nbNeighbors, matches);
			radii[v] = 2.0 * sqrt( pow(matches.back().second,2) / Scalar(nbNeighbors) );
			mAveragePointSpacing += radii[v];
		}

		mAveragePointSpacing /= Scalar( mesh->n_vertices() );
	}

public:

	// RIMLS specific parameters
	void setSigmaR(Scalar v)
	{
		mSigmaR = v;
		mCachedQueryPointIsOK = false;
	}

	void setSigmaN(Scalar v)
	{
		mSigmaN = v;
		mCachedQueryPointIsOK = false;
	}

	void setRefittingThreshold(Scalar v)
	{
		mRefittingThreshold = v;
		mCachedQueryPointIsOK = false;
	}

	void setMinRefittingIters(int n)
	{
		mMinRefittingIters = n;
		mCachedQueryPointIsOK = false;
	}

	void setMaxRefittingIters(int n)
	{
		mMaxRefittingIters = n;
		mCachedQueryPointIsOK = false;
	}

	bool computePotentialAndGradient(const Vector3& x)
	{
		computeNeighborhood(x, true);
		unsigned int nofSamples = mNeighborhood.size();

		if (nofSamples < 1)
		{
			mCachedGradient = Vector3(0);
			mCachedQueryPoint = x;
			mCachedPotential  = 1e9;
			mCachedQueryPointIsOK = false;
			return false;
		}

		if (mCachedRefittingWeights.size() < nofSamples)
			mCachedRefittingWeights.resize( nofSamples + 5 );

		Vector3 source = x;
		Vector3 grad(0);
		Vector3 previousGrad;
		Vector3 sumN(0);
		Scalar potential      = 0.;
		Scalar invSigma2      = Scalar(1) / (mSigmaN*mSigmaN);
		Scalar invSigmaR2     = 0;
		if (mSigmaR>0)
			invSigmaR2 = Scalar(1) / (mSigmaR*mSigmaR);
		Vector3 sumGradWeight;
		Vector3 sumGradWeightPotential;
		Scalar sumW = 0;

		int iterationCount = 0;
		do
		{
			previousGrad = grad;
			sumGradWeight = Vector3(0);
			sumGradWeightPotential = Vector3(0);
			sumN = Vector3(0);
			potential = 0.0;
			sumW = 0.0;

			for (unsigned int i=0; i < nofSamples; i++)
			{
				int id = mNeighborhood[i].first;
				Vertex v(id);
				Vector3 diff = source - points[v];
				Vector3 normal = normals[v];
				Scalar f = dot(diff, normal);

				Scalar refittingWeight = 1;
				if (iterationCount > 0)
				{
					refittingWeight = exp(-(normal - previousGrad).sqrnorm() * invSigma2);
				}
				mCachedRefittingWeights.at(i) = refittingWeight;
				Scalar w = mCachedWeights.at(i) * refittingWeight;
				Vector3 gw = mCachedWeightGradients.at(i) * refittingWeight;

				sumGradWeight += gw;
				sumGradWeightPotential += gw * f;
				sumN += normal * w;
				potential += w * f;
				sumW += w;
			}

			if(sumW == 0.0)
			{
				return false;
			}

			potential /= sumW;
			grad = (-sumGradWeight*potential + sumGradWeightPotential + sumN) * (1.0/sumW);

			iterationCount++;

		} while ( (iterationCount < mMinRefittingIters)
			|| ( (grad - previousGrad).sqrnorm() > mRefittingThreshold && iterationCount < mMaxRefittingIters) );

		mCachedGradient			= grad;
		mCachedPotential		= potential;
		mCachedQueryPoint		= x;
		mCachedQueryPointIsOK	= true;

		mCachedSumGradWeight	= sumGradWeight;
		mCachedSumN				= sumN;
		mCachedSumW				= sumW;
		mCachedSumGradPotential	= sumGradWeightPotential;

		return true;
	}


	bool mlsHessian(const Vector3& x, MatrixType& hessian)
	{
		this->requestSecondDerivatives();
		// at this point we assume computePotentialAndGradient has been called first

		uint nofSamples = mNeighborhood.size();

        const Vector3& sumGradWeight = mCachedSumGradWeight;
		const Scalar& sumW = mCachedSumW;
		const Scalar invW = 1.f/sumW;

		for (uint k = 0; k < 3; ++k)
		{
			Vector3 sumDGradWeight; sumDGradWeight= Vector3(0);
			Vector3 sumDWeightNormal; sumDWeightNormal= Vector3(0);
			Vector3 sumGradWeightNk; sumGradWeightNk= Vector3(0);
			Vector3 sumDGradWeightPotential; sumDGradWeightPotential= Vector3(0);

			for (unsigned int i=0; i<nofSamples; i++)
			{
				int id = mNeighborhood[i].first;
				Vertex v(id);
				Vector3 p = points[v];
				Vector3 diff = x - p;
				Scalar f = dot(diff, normals[v]);

				Vector3 gradW = mCachedWeightGradients.at(i) * mCachedRefittingWeights.at(i);
				Vector3 dGradW = (x-p) * ( mCachedWeightSecondDerivatives.at(i) * (x[k]-p[k]) * mCachedRefittingWeights.at(i));
				dGradW[k] += mCachedWeightDerivatives.at(i);

				sumDGradWeight += dGradW;
				sumDWeightNormal += normals[v] * gradW[k];
				sumGradWeightNk += gradW * normals[v][k];
				sumDGradWeightPotential += dGradW * f;
			}

			Vector3 dGrad = (
				sumDWeightNormal + sumGradWeightNk + sumDGradWeightPotential
				- sumDGradWeight * mCachedPotential
				- sumGradWeight * mCachedGradient[k]
			- mCachedGradient * sumGradWeight[k] ) * invW;

			hessian.col(k) = V2E(dGrad);
		}

		return true;
	}

protected:

	void computeNeighborhood(const Vector3& x, bool computeDerivatives)
	{
		// Find corresponding vertex
		KDResults match;
		mBallTree->ball_search(x, mAveragePointSpacing, match);

		// Find neighborhood
		double r = radii[Vertex(match.front().first)] * mFilterScale;
		mBallTree->ball_search(x, r, mNeighborhood);

		size_t nofSamples = mNeighborhood.size();

		// compute spatial weights and partial derivatives
		mCachedWeights.resize(nofSamples);
		if (computeDerivatives)
		{
			mCachedWeightDerivatives.resize(nofSamples);
			mCachedWeightGradients.resize(nofSamples);
		}
		else
			mCachedWeightGradients.clear();

		for (size_t i = 0; i < nofSamples; i++)
		{
			int id = mNeighborhood[i].first;
			Vertex v(id);
			Scalar s = 1.0 / (radii[v] * mFilterScale);
			s = s*s;
			Scalar w = Scalar(1) - pow(mNeighborhood[i].second, 2) * s;
			if (w<0)
				w = 0;
			Scalar aux = w;
			w = w * w;
			w = w * w;
			mCachedWeights[i] = w;

			if (computeDerivatives)
			{
				mCachedWeightDerivatives[i] = (-2. * s) * (4. * aux * aux * aux);
				mCachedWeightGradients[i]  = (x - points[v]) * mCachedWeightDerivatives[i];
			}
		}
	}

	void requestSecondDerivatives()
	{
		size_t nofSamples = mNeighborhood.size();

		if (nofSamples > mCachedWeightSecondDerivatives.size())
			mCachedWeightSecondDerivatives.resize(nofSamples + 10);

		{
			for (size_t i=0 ; i < nofSamples ; ++i)
			{
				int id = mNeighborhood[i].first;
				Scalar s = 1.0 / (radii[Vertex(id)]*mFilterScale);
				s = s*s;
				Scalar x2 = s * pow(mNeighborhood[i].second, 2);
				x2 = 1.0 - x2;
				if (x2 < 0)
					x2 = 0.;
				mCachedWeightSecondDerivatives[i] = (4.0*s*s) * (12.0 * x2 * x2);
			}
		}
	}
	
protected:

	SurfaceMeshModel * mesh;
	AlignedBox3d mAABB;
	int mGradientHint;
	int mHessianHint;

	NanoKdTree * mBallTree;

	int mMaxNofProjectionIterations;
	Scalar mFilterScale;
	Scalar mAveragePointSpacing;
	Scalar mProjectionAccuracy;

	int mDomainMinNofNeighbors;
	float mDomainRadiusScale;
	float mDomainNormalScale;

	// cached values:
	bool mCachedQueryPointIsOK;
	Vector3 mCachedQueryPoint;
	KDResults mNeighborhood;
	std::vector<Scalar> mCachedWeights;
	std::vector<Scalar> mCachedWeightDerivatives;
	std::vector<Vector3> mCachedWeightGradients;
	std::vector<Scalar> mCachedWeightSecondDerivatives;

	// RIMLS specific
	int mMinRefittingIters;
	int mMaxRefittingIters;
	Scalar mRefittingThreshold;
	Scalar mSigmaN;
	Scalar mSigmaR;

	// cached values:
	Vector3 mCachedGradient;
	Scalar mCachedPotential;

	Scalar mCachedSumW;
	std::vector<Scalar> mCachedRefittingWeights;;
	Vector3 mCachedSumN;
	Vector3 mCachedSumGradWeight;
	Vector3 mCachedSumGradPotential;
};
