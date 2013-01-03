#include "ARAPCurveDeformer.h"
using namespace SurfaceMeshTypes;

ARAPCurveDeformer::ARAPCurveDeformer( std::vector<Vec3d> curvePoints, int sizeNeighborhood )
{
    isSolverReady = false;

    this->points = curvePoints;
    this->nVerts = points.size();
    this->isAnchorPoint = std::vector< bool >(false, nVerts);
    this->isControlPoint = std::vector< bool >(false, nVerts);
	this->neighborhoodSize = qMax(1, sizeNeighborhood);

	ClearAll();
}

void ARAPCurveDeformer::ComputeCotWeights()
{
    //Point p, p1, p2, p3;
    //int pid, p1id, p2id, p3id;

    double wij = 0;

    for (int vit = 0; vit < nVerts; vit++)
    {
        // find alpha and beta (LSO paper), then calculate cotangent weights
        // alpha is P_P1_P2, beta is P_P3_P2

		foreach(int j, neighbors(vit))
		{
			wij = 0;

			// Weight choices:
			//wij = 1.0;								// uniform weight
			wij = (points[j] - points[vit]).norm();		// Edge length as weight
			//wij =										// Angle as weight

			wij_weight[vit][j] = wij / 2.0;
		}
    }
}

void ARAPCurveDeformer::BuildAndFactor(){
    ComputeCotWeights();

    // Initialize
    xyz.clear();		xyz.resize(3, VectorXd::Zero(nVerts));		// Store new vertex positions
    b.clear();			b.resize(3, VectorXd::Zero(nVerts));		// for LHS
	R.clear();			R.resize(nVerts, Matrix3d::Identity());		// No rotation
    OrigMesh.clear();	OrigMesh.resize(nVerts, Vector3d::Zero());

    // L matrix, n by n, cotangent weights
    typedef Eigen::Triplet<Scalar> T;
    std::vector< T > L;

    for (int vit = 0; vit < nVerts; vit++)
    {
        int i = vit;

        OrigMesh[i] = Vector3d(points[vit][0], points[vit][1], points[vit][2]);

        double weight = 0.0;

        if(!(isAnchorPoint[vit] || isControlPoint[vit]))
        {
            // Over neighbors
			foreach(int j, neighbors(vit))
			{
				weight += wij_weight[vit][j];
				L.push_back( T(i, j, -wij_weight[vit][j]) );
			}
        }
        else
        {
            weight = 1.0;
        }

        L.push_back( T(i, i, weight) );
    }

    SparseMatrix<double> A(nVerts, nVerts);
    A.setFromTriplets(L.begin(), L.end());

    At = A.transpose();

    // FACTOR:
    solver.compute(At * A);

    isSolverReady = true;
}

void ARAPCurveDeformer::SVDRotation()
{
    Matrix3d eye = Matrix3d::Identity();

    for (int vit = 0; vit < nVerts; vit++)
    {
        int i = Surface_mesh::Vertex(vit).idx();

		std::vector<int> adj = neighbors(vit);

        int valence = adj.size(); // its a curve, only two neighbors at most
        int degree = 0;

        MatrixXd P(3, valence), Q(3, valence);

        // Over neighbors
		foreach( int j, adj )
		{
			P.col(degree)	= (OrigMesh[i] - OrigMesh[j]) * wij_weight[vit][j];
			Q.col(degree++) = (	Vector3d(xyz[0][i], xyz[1][i], xyz[2][i]) -
								Vector3d(xyz[0][j], xyz[1][j], xyz[2][j]));
		}

        // Compute the 3 by 3 covariance matrix:
        // actually S = (P * W * Q.t()); W is already considerred in the previous step (P=P*W)
        MatrixXd S = (P * Q.transpose());

        // Compute the singular value decomposition S = UDV.t
        JacobiSVD<MatrixXd> svd(S, ComputeThinU | ComputeThinV); // X = U * D * V.t()

        MatrixXd V = svd.matrixV();
        MatrixXd Ut = svd.matrixU().transpose();

        eye(2,2) = (V * Ut).determinant();	// remember: Eigen starts from zero index

        // V*U.t may be reflection (determinant = -1). in this case, we need to change the sign of
        // column of U corresponding to the smallest singular value (3rd column)
        R[i] = (V * eye * Ut); //Ri = (V * eye * U.t());
    }
}

void ARAPCurveDeformer::Deform( int ARAPIteration /*= 1*/ )
{
    if(!isSolverReady)
        BuildAndFactor();

    // ARAP iteration
    for(int iter = 0; iter <= ARAPIteration; iter++)
    {
        // update vector b3 = wij/2 * (Ri+Rj) * (pi - pj), where pi and pj are coordinates of the original mesh
        for (int vit = 0; vit < nVerts; vit++)
        {
            int i = vit;

            Vector3d p (points[vit][0], points[vit][1], points[vit][2]);

            if(!(isAnchorPoint[vit] || isControlPoint[vit]))
            {
                p = Vector3d::Zero(); // Set to zero

                // Collect neighbors
				foreach( int j, neighbors(vit) )
				{
					Vector3d pij = OrigMesh[i] - OrigMesh[j];
					Vector3d RijPijMat = ((R[i] + R[j]) * pij);

					p += RijPijMat * (wij_weight[vit][j] / 2.0);
				}
            }

            // Set RHS
            for(int k = 0; k < 3; k++)
                b[k][i] = p[k];
        }

        // SOLVE for x, y, and z
        for(int k = 0; k < 3; k++)
            xyz[k] = solver.solve(At * b[k]);

        // if iter = 0, just means naive Laplacian Surface Editing (Ri is identity matrix)
        if(iter > 0) SVDRotation();
    }

    // update vertex coordinates
    for (int vit = 0; vit < nVerts; vit++)
    {
        int i = Surface_mesh::Vertex(vit).idx();
        points[vit] = Vec3d (xyz[0][i], xyz[1][i], xyz[2][i]);
    }
}

std::vector<int> ARAPCurveDeformer::neighbors( int idx )
{
	std::vector<int> adj;

	for(int i = 1; i < neighborhoodSize + 1; i++)
	{
		int prev = idx - i;
		int next = idx + i;
		if(prev >= 0) adj.push_back(prev);
		if(next < nVerts) adj.push_back(next);
	}

	assert(adj.size() > 0);

	return adj;
}
