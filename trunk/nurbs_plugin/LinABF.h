#pragma once
// Based on code by Rhaleb Zayer

#include "SurfaceMeshModel.h"
#include "SurfaceMeshHelper.h"

using namespace SurfaceMeshTypes;

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
using namespace Eigen;
typedef Eigen::Triplet<Scalar> T;

#define M_PI    3.14159265358979323846
#define M_PI_2  1.57079632679489661923
#define cot(x)	(tan(M_PI_2 - x))

struct LinABF{

	LinABF( SurfaceMeshModel * useMesh )
	{
		if(!useMesh->is_triangle_mesh()){
			qDebug() << "WARNING: mesh has been triangulated!";
			useMesh->triangulate();
		}

		this->mesh = useMesh;
		this->points = mesh->vertex_property<Vector3>(VPOINT);

		this->np = mesh->n_vertices();
		this->nt = mesh->n_faces();
		this->nA = 3 * nt;

		/// 1) Find boundary / internal vertices
		foreach(Vertex v, mesh->vertices()){
			if(mesh->is_boundary(v))
				b.insert( v.idx() );
			else
				internal.push_back( v );
		}

		fv.resize( nt );
		fv123.reserve( 3 * nt );
		ang123 = Eigen::VectorXd::Zero( 3 * nt );

		/// 2) Angle pre-processing
		foreach(Face f, mesh->faces())
		{
			// Collect vertices of face
			Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
			do{ fv[f.idx()].push_back( Vertex(vit) ); } while(++vit != vend);
			std::vector<Vertex> & vface = fv[f.idx()];

			Vector3 r23 = (points[vface[2]] - points[vface[1]]).normalized();
			Vector3 r31 = (points[vface[0]] - points[vface[2]]).normalized();
			Vector3 r12 = (points[vface[1]] - points[vface[0]]).normalized();

			int i = f.idx() * 3;
			ang123[i + 0] = acos(-dot(r12, r31));
			ang123[i + 1] = acos(-dot(r12, r23));
			ang123[i + 2] = acos(-dot(r31, r23));

			fv123.push_back(vface[0]);	fv123.push_back(vface[1]);	fv123.push_back(vface[2]);
			fv231.push_back(vface[1]);	fv231.push_back(vface[2]);	fv231.push_back(vface[0]);
		}

		/// 3) Update with optimal angles:

		// Fill with corresponding ones
		std::vector< T > Mang_T;
		foreach(Face f, mesh->faces()){
			int i = f.idx() * 3;
			Mang_T.push_back( T(fv123[i+0].idx(), fv231[i+0].idx(), ang123[i+0]) );
			Mang_T.push_back( T(fv123[i+1].idx(), fv231[i+1].idx(), ang123[i+1]) );
			Mang_T.push_back( T(fv123[i+2].idx(), fv231[i+2].idx(), ang123[i+2]) );
		}

		// Ring angles
		Mang = Eigen::SparseMatrix<Scalar,Eigen::RowMajor>(np,np);
		Mang.setFromTriplets( Mang_T.begin(), Mang_T.end() );

		// Sum it up
		Eigen::VectorXd sumringangles(np);
		for(int i = 0; i < np; i++) sumringangles(i) = Mang.row(i).sum();

		// Optimal angles
		foreach(Face f, mesh->faces())
		{
			int i = f.idx() * 3;

			double ang1opt = 2 * M_PI * ang123[i+0] / sumringangles[ fv123[i+0].idx() ];
			double ang2opt = 2 * M_PI * ang123[i+1] / sumringangles[ fv123[i+1].idx() ];
			double ang3opt = 2 * M_PI * ang123[i+2] / sumringangles[ fv123[i+2].idx() ];

			ang123opt.push_back(ang1opt);	ang123opt.push_back(ang2opt);	ang123opt.push_back(ang3opt);
		}

		// Find angles with big deficit
		rhsk = Eigen::VectorXd::Constant(np, 2 * M_PI) - sumringangles;

		// Threshold 1.0
		QSet<int> icha;
		for(int i = 0; i < (int)rhsk.rows(); i++) if(abs(rhsk(i)) > 1.0) icha.insert(i);

		// Exclude boundary
		QSet<int> icha_internal = icha - b;
		//std::set_difference(icha.begin(), icha.end(), b.begin(), b.end(), std::inserter(icha_internal, icha_internal.end()));

		// Go over all ang123 and replace 'vi' values with optimal
		foreach(int vi, icha_internal){
			foreach(Face f, mesh->faces()){
				int fidx = f.idx();

				int v1 = 3*fidx + 0;
				int v2 = 3*fidx + 1;
				int v3 = 3*fidx + 2;

				if(vi == fv[fidx][0].idx()) ang123[v1] = ang123opt[v1];
				if(vi == fv[fidx][1].idx()) ang123[v2] = ang123opt[v2];
				if(vi == fv[fidx][2].idx()) ang123[v3] = ang123opt[v3];
			}
		}

		/// 4) Fill condition matrices
		std::vector< T > B1_T, B2_T, B3_T, tmp_T;

		// B1:
		rhs1 = Eigen::VectorXd(nt);
		for(int i = 0; i < nt; i++){
			B1_T.push_back(T(i,3*i+0, 1));
			B1_T.push_back(T(i,3*i+1, 1));
			B1_T.push_back(T(i,3*i+2, 1));

			// Sum angles
			Scalar sum = ang123[3*i+0] + ang123[3*i+1] + ang123[3*i+2];
			rhs1(i) = M_PI - sum;
		}
		B1 = Eigen::SparseMatrix<Scalar>(nt,nA);
		B1.setFromTriplets( B1_T.begin(), B1_T.end() );
		B1_T.clear();

		// B2:
		for(int idx = 0; idx < nt; idx++)
		{
			int a1 = fv[idx][0].idx();
			int a2 = fv[idx][1].idx();
			int a3 = fv[idx][2].idx();

			int it0 = 3*idx + 0;
			int it1 = 3*idx + 1;
			int it2 = 3*idx + 2;

			if(!b.contains(a1)) tmp_T.push_back( T(a1,it0, 1) );
			if(!b.contains(a2)) tmp_T.push_back( T(a2,it1, 1) );
			if(!b.contains(a3)) tmp_T.push_back( T(a3,it2, 1) );
		}

		// Only compute for internals
		{
			std::set<int> usedRow;
			std::map<int,int> rowMap;
			foreach(T t, tmp_T) usedRow.insert(t.row());
			foreach(int r, usedRow) rowMap[r] = rowMap.size();
			foreach(T t, tmp_T) B2_T.push_back( T(rowMap[t.row()],t.col(),t.value()) );
		}

		B2 = Eigen::SparseMatrix<Scalar>(np - b.size(), nA);
		B2.setFromTriplets( B2_T.begin(), B2_T.end() );
		B2_T.clear();

		Eigen::VectorXd b2_ang123 = B2 * ang123;
		Eigen::VectorXd two_pi = Eigen::VectorXd::Constant(b2_ang123.rows(), 2 * M_PI);
		rhs2 = two_pi - b2_ang123;

		// Side condition
		// B3:
		tmp_T.clear();
		for(int idx = 0; idx < nt; idx++)
		{
			int a1 = fv[idx][0].idx();
			int a2 = fv[idx][1].idx();
			int a3 = fv[idx][2].idx();

			int it0 = 3*idx + 0;
			int it1 = 3*idx + 1;
			int it2 = 3*idx + 2;

			if(!b.contains(a1)){
				tmp_T.push_back( T(a1,it2, cot( ang123(it2) )) );
				tmp_T.push_back( T(a1,it1, -cot( ang123(it1) )) );
			}

			if(!b.contains(a2)){
				tmp_T.push_back( T(a2,it0, cot( ang123(it0) )) );
				tmp_T.push_back( T(a2,it2, -cot( ang123(it2) )) );
			}

			if(!b.contains(a3)){
				tmp_T.push_back( T(a3,it1, cot( ang123(it1) )) );
				tmp_T.push_back( T(a3,it0, -cot( ang123(it0) )) );
			}
		}

		// Only compute for internals
		{
			std::set<int> usedRow;
			std::map<int,int> rowMap;
			foreach(T t, tmp_T) usedRow.insert(t.row());
			foreach(int r, usedRow) rowMap[r] = rowMap.size();
			foreach(T t, tmp_T) B3_T.push_back( T(rowMap[ t.row() ],t.col(),t.value()) );
		}

		B3 = Eigen::SparseMatrix<Scalar>(np - b.size(), nA);
		B3.setFromTriplets( B3_T.begin(), B3_T.end() );
		B3_T.clear();

		// CC:
		tmp_T.clear();
		for(int idx = 0; idx < nt; idx++)
		{
			int a1 = fv[idx][0].idx();
			int a2 = fv[idx][1].idx();
			int a3 = fv[idx][2].idx();

			int it0 = 3*idx + 0;
			int it1 = 3*idx + 1;
			int it2 = 3*idx + 2;

			double vs0 = log( sin(ang123(it0)) );
			double vs1 = log( sin(ang123(it1)) );
			double vs2 = log( sin(ang123(it2)) );

			tmp_T.push_back( T(a1,it2, -vs2 ) );
			tmp_T.push_back( T(a1,it1,  vs1 ) );

			tmp_T.push_back( T(a2,it0, -vs0) );
			tmp_T.push_back( T(a2,it2,  vs2) );

			tmp_T.push_back( T(a3,it1, -vs1) );
			tmp_T.push_back( T(a3,it0,  vs0) );
		}
		CC = Eigen::SparseMatrix<Scalar,Eigen::RowMajor>(np, nA);
		CC.setFromTriplets( tmp_T.begin(), tmp_T.end() );
		tmp_T.clear();

		Eigen::VectorXd sumCC(np);
		for(int i = 0; i < np; i++) sumCC(i) = CC.row(i).sum();

		// Fill 'rhs3' with only sum for internals
		rhs3 = Eigen::VectorXd::Zero(np - b.size());
		for(int i = 0, j = 0; i < np; i++) if(!b.contains(i)) rhs3(j++) = sumCC(i); 

		// Gather all
		int b1 = B1.rows(), b2 = B2.rows(), b3 = B3.rows();
		Eigen::SparseMatrix<Scalar,Eigen::RowMajor> M = Eigen::SparseMatrix<Scalar,Eigen::RowMajor>(b1 + b2 + b3, nA);
		M.middleRows(0		, b1) = B1;
		M.middleRows(b1		, b2) = B2;
		M.middleRows(b1 + b2, b3) = B3;

		// For change of variables
		tmp_T.clear(); for(int i = 0; i < nA; i++) tmp_T.push_back(T(i,i,ang123(i)));
		W = Eigen::SparseMatrix<Scalar>(nA,nA);
		W.setFromTriplets(tmp_T.begin(),tmp_T.end()); tmp_T.clear();

		// Matrix A:
		A = M * W;
		At = A.transpose();

		// Vector rhs:
		int j = 0;
		rhs = Eigen::VectorXd::Zero( rhs1.size() + rhs2.size() + rhs3.size() );
		for(int i = 0; i < rhs1.size(); i++) rhs(j++) = rhs1(i);
		for(int i = 0; i < rhs2.size(); i++) rhs(j++) = rhs2(i);
		for(int i = 0; i < rhs3.size(); i++) rhs(j++) = rhs3(i);

		// Solve:
		Eigen::CholmodSupernodalLLT< Eigen::SparseMatrix<Scalar> > solver(A * At);
		Eigen::VectorXd ee = At * solver.solve( rhs );
		ee = W * ee;
		angsol = ang123 + ee;

		solution.resize(nt,3);
		for(int i = 0; i < nt; i++)
		{
			solution(i,0) = angsol(3*i + 0);
			solution(i,1) = angsol(3*i + 1);
			solution(i,2) = angsol(3*i + 2);
		}

		/// 5) Get positions from angles
		Eigen::VectorXd coef(nt), a(nt), b(nt);

		for(int i = 0; i < nt; i++)
		{
			coef(i) = sin(solution(i,1)) / sin(solution(i,2));
			a(i) = coef(i) * cos( solution(i,0) );
			b(i) = coef(i) * sin( solution(i,0) );
		}

		std::vector< T > MV_T;
		for(int i = 0; i < nt; i++)
		{
			int a1 = fv[i][0].idx();
			int a2 = fv[i][1].idx();
			int a3 = fv[i][2].idx();

			MV_T.push_back( T(i, a1		, 1-a(i) ) );
			MV_T.push_back( T(i, a1+np	,	b(i) ) );
			MV_T.push_back( T(i, a2		,	a(i) ) );
			MV_T.push_back( T(i, a2+np	,  -b(i) ) );
			MV_T.push_back( T(i, a3		,	 -1  ) );

			int j = i + nt;

			MV_T.push_back( T(j, a1		,  -b(i) ) );
			MV_T.push_back( T(j, a1+np	, 1-a(i) ) );
			MV_T.push_back( T(j, a2		,	b(i) ) );
			MV_T.push_back( T(j, a2+np	,   a(i) ) );
			MV_T.push_back( T(j, a3+np	,	 -1  ) );
		}

		MV = Eigen::SparseMatrix<Scalar,Eigen::RowMajor>(2*nt,2*np);
		MV.setFromTriplets( MV_T.begin(), MV_T.end() );
		MVt = MV.transpose();
		MV = MVt * MV;

		// Pinned vertices conditions
		int a1 = fv[0][0].idx(), a2 = fv[0][1].idx();
		int pinned[] = { a1, a2, a1+np, a2+np };

		Eigen::SparseMatrix<Scalar,Eigen::RowMajor> MV_pinned(4,2*np);
		for(int i = 0; i < 4; i++) MV_pinned.insert(i,pinned[i]) = 1;
		for(int i = 0; i < 4; i++) MV.middleRows(pinned[i],1) = MV_pinned.row(i);

		uvc = Eigen::VectorXd::Zero( 2 * np );
		uvc( pinned[1] ) = 1.0;

		MVt = MV.transpose();
		Eigen::CholmodSupernodalLLT< Eigen::SparseMatrix<Scalar> > postion_solver( MVt * MV );
		Eigen::VectorXd uv = postion_solver.solve( MVt * uvc );

		// Save final UV coordinates
		this->tex_coord = mesh->vertex_property<Vec2d>("v:texture", Vec2d(0,0));
		foreach(Vertex v, mesh->vertices())
		{
			int i = v.idx();
			tex_coord[v] = Vec2d( uv(i), uv(i+np) );
		}
	}

	void applyUVToMesh()
	{
		foreach(Vertex v, mesh->vertices())
			points[v] = Vector3(tex_coord[v][0],tex_coord[v][1],0);
	}

	SurfaceMeshModel * mesh;
	Vector3VertexProperty points;

	int np, nt;								// Number of points / triangles
	int nA;									// Number of angles on triangular mesh
	QSet<int> b;							// Boundary vertices
	std::vector<Vertex> internal;			// Internal vertices

	std::vector< std::vector<Vertex> > fv;	// Vertex indices per face
	std::vector<Vertex> fv123;				// Flat version of fv
	std::vector<Vertex> fv231;				// Re-ordered version of fv123
	Eigen::VectorXd ang123, angsol;			// Angles of all faces f_ang1,f_ang2,f_ang3,..
	std::vector<Scalar> ang123opt;			// Optimal angles
	std::vector<Scalar> ang123sum;			// Sum of angles

	Eigen::SparseMatrix<Scalar,Eigen::RowMajor> Mang;
	Eigen::VectorXd rhsk;

	Eigen::SparseMatrix<Scalar> B1,B2,B3;	// Parts of system matrix 'A'
	Eigen::VectorXd rhs1, rhs2, rhs3;		// Right hand side
	Eigen::SparseMatrix<Scalar,Eigen::RowMajor> CC;
	Eigen::SparseMatrix<Scalar> W;

	// System
	Eigen::SparseMatrix<Scalar> A, At;
	Eigen::VectorXd rhs;
	Eigen::MatrixXd solution;

	Eigen::SparseMatrix<Scalar,Eigen::RowMajor> MV, MVt;
	Eigen::MatrixXd uvc;

	// Output
	SurfaceMeshModel::Vertex_property<Vec2d> tex_coord;
};
