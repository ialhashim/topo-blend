#include "FFD.h"

FFD::FFD( const std::vector<Vector3d> & pointSoup, FFD_FitType fit_type, Vector3i res )
{
	foreach(Vector3d p, pointSoup)	temp_mesh.add_vertex(p);
	init(&temp_mesh,fit_type,res);
}

FFD::FFD( Surface_mesh * src_mesh, FFD_FitType fit_type, Vector3i res )
{
	init(src_mesh,fit_type,res);
}

void FFD::init( Surface_mesh * src_mesh, FFD_FitType fit_type, Vector3i res )
{
	this->mesh = src_mesh;

	if(!mesh) return;

	// Mesh dimensions
	Eigen::AlignedBox3d box;

	mesh_points = mesh->vertex_property<Vector3d>("v:point");
	Surface_mesh::Vertex_iterator vit, vend = mesh->vertices_end();

	for (vit = mesh->vertices_begin(); vit != vend; ++vit)
		box = box.merged( Eigen::AlignedBox3d(mesh_points[vit], mesh_points[vit])  );

	width = box.max().x() - box.min().x();
	length = box.max().y() - box.min().y();
	height = box.max().z() - box.min().z();

	// Deal with thin / flat cases
	double nonZero = 0;
	if		(width	== 0) nonZero = qMin(length, height);
	else if (length == 0) nonZero = qMin(width, height);
	else if (height == 0) nonZero = qMin(width, length);
	if(nonZero == 0) nonZero = (length + height + width) * 0.01;

	if(width == 0) width = nonZero;
	if(length == 0) length = nonZero;
	if(height == 0) height = nonZero;

	center = (box.max() + box.min()) * 0.5;

	// Expand a bit so all the vertices are in (0,1)
	double radius = 0.5 * box.diagonal().norm();
	width += radius * 0.05;
	length += radius * 0.05;
	height += radius * 0.05;

	// Fit to bounding box (default)
	if(fit_type == BoundingBoxFFD)
		bbFit(res);
}

void FFD::bbFit( Vector3i res )
{
    int Nx = qMax(2, res.x());
    int Ny = qMax(2, res.y());
    int Nz = qMax(2, res.z());

	this->resolution = Vector3i(Nx, Ny, Nz);

	double dx = width / (Nx-1);
	double dy = length / (Ny-1);
	double dz = height / (Nz-1);

	Vector3d start_corner(-width/2, -length/2, -height/2);

	start_corner += center;

	// indexing
	int i = 0;

	// Nx x Ny x Nz
    pointsGridIdx = std::vector<std::vector<std::vector < int > > >
        (Nx, std::vector< std::vector < int > >(Ny, std::vector < int >(Nz)));

	for(int z = 0; z < Nz; z++){
		for(int y = 0; y < Ny; y++){
			for(int x = 0; x < Nx; x++){
				// Grid indexing
				pointsGridIdx[x][y][z] = i;

				// Control point position
				Vector3d p = start_corner + Vector3d(dx * x, dy * y, dz * z);

				// Add it
                control_points.push_back(new ControlPoint(p, i++, Vector3i(x,y,z)));
			}
		}	
	}

	// Setup local coordinate
	mP = start_corner; // this is the origin of our local frame
	mS = width * Vector3d(1,0,0);
	mT = length * Vector3d(0,1,0);
	mU = height * Vector3d(0,0,1);

	// Get copy of original mesh vertices in local coordinates
    Surface_mesh::Vertex_iterator vit, vend = mesh->vertices_end();

	meshVerticesLocal.reserve( mesh->n_vertices() );

	for (vit = mesh->vertices_begin(); vit != vend; ++vit){
		meshVerticesLocal.push_back( getLocalCoordinates(mesh_points[vit]) );
	}
}

void FFD::apply()
{
	Surface_mesh::Vertex_iterator vit, vend = mesh->vertices_end();

	for (vit = mesh->vertices_begin(); vit != vend; ++vit)
	{
		int vidx = ((Surface_mesh::Vertex)vit).idx();

		mesh_points[vit] = getWorldCoordinate( deformVertexLocal(meshVerticesLocal[vidx]) );
	}
}

Vector3d FFD::outputPoint( int idx )
{
	return mesh_points[ Surface_mesh::Vertex(idx) ];
}

std::vector<Vector3d> FFD::outputPoints()
{
	std::vector<Vector3d> deformedPoints;
    for(int i = 0; i < (int)mesh->n_vertices(); i++)
		deformedPoints.push_back( outputPoint(i) );
	return deformedPoints;
}

void FFD::customVolume( Vector3i res, Vector3d location, double spacing, std::map<int,Vector3d> pnts )
{
	int Nx = qMax(2, res.x());
	int Ny = qMax(2, res.y());
	int Nz = qMax(2, res.z());

	this->resolution = Vector3i(Nx, Ny, Nz);

	width = length = height = spacing;

	double dx = width / (Nx-1);
	double dy = length / (Ny-1);
	double dz = height / (Nz-1);

	Vector3d start_corner(-width/2, -length/2, -height/2);

	start_corner += location;

	// indexing
	int i = 0;

	// Nx x Ny x Nz
    pointsGridIdx = std::vector<std::vector<std::vector < int > > >
        (Nx, std::vector< std::vector < int > >(Ny, std::vector < int >(Nz)));

	for(int z = 0; z < Nz; z++){
		for(int y = 0; y < Ny; y++){
			for(int x = 0; x < Nx; x++){
				// Grid indexing
				pointsGridIdx[x][y][z] = i;

				// Control point position
				Vector3d p = start_corner + Vector3d(dx * x, dy * y, dz * z);

				// Add it
				control_points.push_back(new ControlPoint(p, i++, Vector3i(x,y,z)));
			}
		}	
	}

	// Setup local coordinate
	mP = start_corner; // this is the origin of our local frame
	mS = spacing * Vector3d(1,0,0);
	mT = spacing * Vector3d(0,1,0);
	mU = spacing * Vector3d(0,0,1);

	// Get copy of original mesh vertices in local coordinates
    for(std::map<int,Vector3d>::iterator it = pnts.begin(); it != pnts.end(); it++)
	{
		int vid = it->first;
		Vector3d pos = it->second;
		fixedPointsLocal[vid] = getLocalCoordinates(pos);
	}
}

std::map<int,Vector3d> FFD::applyCustom()
{
    std::map<int,Vector3d> deformed;

    for(std::map<int,Vector3d>::iterator it = fixedPointsLocal.begin(); it != fixedPointsLocal.end(); it++)
	{
		deformed[it->first] = getWorldCoordinate( deformVertexLocal(it->second) );
	}

	return deformed;
}

Vector3d FFD::deformVertexLocal( const Vector3d & localPoint )
{
	Vector3d newVertex (0,0,0);

	double s = localPoint.x();
	double t = localPoint.y();
	double u = localPoint.z();

	int S = resolution.x()-1, T = resolution.y()-1, U = resolution.z()-1;

    // From Explicit definition of Bezier curves
	for (int k = 0; k <= U; k++)
	{
		int ck = Coef(U,k);
		double uk = pow(u,k) * pow(1-u, U-k);

		for (int j = 0; j <= T; j++)
		{
			int cj = Coef(T,j);
			double tj = pow(t,j) * pow(1-t, T-j);

			for (int i = 0; i <= S; i++)
			{
				int ci =  Coef(S,i);
				double si = pow(s,i) * pow(1-s, S-i);

				Vector3d controlPointPos = getLocalCoordinates(*control_points[pointsGridIdx[i][j][k]]);

				// as combination
				newVertex += ci*cj*ck*si*tj*uk * (controlPointPos);
			}
		}
	}

	return newVertex;
}

Vector3d FFD::getWorldCoordinate(const Vector3d & pLocal)
{
	return mP + pLocal.x()*mS + pLocal.y()*mT + pLocal.z()*mU;
}

Vector3d FFD::getLocalCoordinates( const Vector3d & p )
{
	Vector3d V = p;
	double s = 0, t = 0, u = 0;

	Vector3d TXU = mT.cross(mU);
	s = TXU.dot(V - mP) / TXU.dot(mS);

	Vector3d SXU = mS.cross(mU);
	t = SXU.dot(V - mP) / SXU.dot(mT);

	Vector3d SXT = mS.cross(mT);
	u = SXT.dot(V - mP) / SXT.dot(mU);

	return Vector3d(s,t,u);
}
