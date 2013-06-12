#include <QElapsedTimer>

#include "DynamicVoxel.h"

#include "SurfaceMeshModel.h"
#include "SurfaceMeshHelper.h"
using namespace SurfaceMesh;

#include "DoubleTupleMap.h"

#include "weld.h"
#include "PolygonArea.h"

using namespace DynamicVoxelLib;

// Eigen Library
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
using namespace Eigen;
typedef CholmodSupernodalLLT< SparseMatrix<double> > CholmodSolver;

DynamicVoxel::DynamicVoxel(double voxelSize){

    this->voxel_size = voxelSize;

    this->maxVoxel = Voxel(-INT_MAX, -INT_MAX, -INT_MAX);
    this->minVoxel = Voxel(INT_MAX, INT_MAX, INT_MAX);
}

void DynamicVoxel::draw(){

    glLineWidth(2);

    foreach(Voxel v, voxels)
    {
        glColor3d(0,0,0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        drawCube(v.x, v.y, v.z, voxel_size);

        glColor3d(0.5,0,0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        drawCube(v.x, v.y, v.z, voxel_size);
    }
}

std::vector<Voxel> DynamicVoxel::voxelSphere(double radius)
{
    std::vector<Voxel> sphere;

    int steps = (int)(radius / voxel_size);

    for(int x = 0; x < steps; x++){
        for(int y = 0; y < steps; y++){
            for(int z = 0; z < steps; z++){
                double X = x * voxel_size;
                double Y = y * voxel_size;
                double Z = z * voxel_size;

                double r = sqrt(X*X + Y*Y + Z*Z);

                if(r <= radius /*&& r >= (radius - voxel_size*2)*/){
                    sphere.push_back(Voxel( x, y, z));
                    sphere.push_back(Voxel(-x, y, z));
                    sphere.push_back(Voxel( x,-y, z));
                    sphere.push_back(Voxel(-x,-y, z));
                    sphere.push_back(Voxel( x, y,-z));
                    sphere.push_back(Voxel(-x, y,-z));
                    sphere.push_back(Voxel( x,-y,-z));
                    sphere.push_back(Voxel(-x,-y,-z));
                }
            }
        }
    }

    return sphere;
}

std::vector<Voxel> DynamicVoxel::voxelTorus(double pathRadius, double circleRadius)
{
    std::vector<Voxel> torus;

    int stepsXY = (int)((pathRadius+circleRadius) / voxel_size);
    int stepZ = (int)(circleRadius*2 / voxel_size);

    double R = pathRadius;
    double r = circleRadius;

    for(int x = 0; x < stepsXY; x++){
        for(int y = 0; y < stepsXY; y++){
            for(int z = 0; z < stepZ; z++){

                double X = x * voxel_size;
                double Y = y * voxel_size;
                double Z = z * voxel_size;

                double val = pow(R - sqrt(X*X + Y*Y), 2) + Z*Z;

                if(val < r*r)
                {
                    torus.push_back(Voxel( x, y, z));
                    torus.push_back(Voxel(-x, y, z));
                    torus.push_back(Voxel( x,-y, z));
                    torus.push_back(Voxel(-x,-y, z));
                    torus.push_back(Voxel( x, y,-z));
                    torus.push_back(Voxel(-x, y,-z));
                    torus.push_back(Voxel( x,-y,-z));
                    torus.push_back(Voxel(-x,-y,-z));
                }
            }
        }
    }

    return torus;
}

std::vector<Voxel> DynamicVoxel::voxelCircle(double radius)
{
    std::vector<Voxel> circle;

    int r = radius*0.5 / voxel_size;
    int xm = 0, ym = 0;

    int x = -r, y = 0, err = 2-2*r; /* II. Quadrant */
    do {
        circle.push_back(Voxel(xm-x, ym+y)); /*   I. Quadrant */
        circle.push_back(Voxel(xm-y, ym-x)); /*  II. Quadrant */
        circle.push_back(Voxel(xm+x, ym-y)); /* III. Quadrant */
        circle.push_back(Voxel(xm+y, ym+x)); /*  IV. Quadrant */
        r = err;
        if (r <= y) err += ++y*2+1;
        if (r > x || err > y) err += ++x*2+1;
    } while (x < 0);

    return circle;
}

std::vector<Voxel> DynamicVoxel::orientedVoxelCircle(double radius, const Vector3d &direction)
{
    std::vector<Voxel> result;
    std::vector<Voxel> sphere = voxelSphere(radius*2);

    foreach(Voxel voxel, sphere){
        Vector3d v(voxel.x, voxel.y, voxel.z);
        v /= 2.0;

        if(abs(dot(direction, v.normalized())) < voxel_size)
            result.push_back(Voxel(v.x(), v.y(), v.z()));
    }

    return result;
}

std::vector<Voxel> DynamicVoxel::voxelLine(const Vector3d &p1, const Vector3d &p2, bool thick)
{
    std::vector<Voxel> line;

    Vector3d d = (p2 - p1) / voxel_size;
    double N = qMax( abs(d.x()), qMax(abs(d.y()), abs(d.z())) );
    Voxel s = Voxel(Vector3d(d / N));

    Voxel p = Voxel(Vector3d(p1 / voxel_size));

    line.push_back( p );

    for(int i = 0; i < N; i++){
        p = p + s;
        line.push_back( p );

        // Over sampling
        if(thick)
        {
            line.push_back( Voxel(p) + Voxel(1,0,0) );
            line.push_back( Voxel(p) + Voxel(0,1,0) );
            line.push_back( Voxel(p) + Voxel(0,0,1) );
            line.push_back( Voxel(p) + Voxel(0,0,-1) );
            line.push_back( Voxel(p) + Voxel(0,-1,0) );
            line.push_back( Voxel(p) + Voxel(-1,0,0) );
        }
    }

    return line;
}

void DynamicVoxel::addLine(const Vector3d &p1, const Vector3d &p2)
{
    std::vector<Voxel> line = voxelLine(p1, p2);

    foreach(Voxel v, line)
        setVoxel( v.x, v.y, v.z );
}

void DynamicVoxel::addCircle(const Vector3d &center, double radius, const Vector3d &direction)
{
    std::vector<Voxel> circle3D = orientedVoxelCircle(radius, direction);

    foreach(Voxel voxel, circle3D){
        Vector3d v(voxel.x, voxel.y, voxel.z);
        v += center / voxel_size;
        setVoxel( v.x(), v.y(), v.z() );
    }
}

void DynamicVoxel::addSphere(const Vector3d &center, double radius)
{
    // Recenter
    foreach(Voxel voxel, voxelSphere(radius)){
        Vector3d v(voxel.x, voxel.y, voxel.z);
        v += Vector3d(center / voxel_size);
        setVoxel( v.x(), v.y(), v.z() );
    }
}

void DynamicVoxel::addHemiSphere(const Vector3d &center, double radius, const Vector3d & direction)
{
    std::vector<Voxel> hemisphere = voxelSphere(radius);

    // Recenter
    foreach(Voxel voxel, hemisphere){
        Vector3d v(voxel.x, voxel.y, voxel.z);

        if(dot(direction, v) > 0)
        {
            v += center / voxel_size;
            setVoxel( v.x(), v.y(), v.z() );
        }
    }
}

void DynamicVoxel::addCylinder(const Vector3d &from, const Vector3d &to, double radius){
    Vector3d direction = (from - to).normalized();

    std::vector<Voxel> cross_section = orientedVoxelCircle(radius, direction);
    std::vector<Voxel> path = voxelLine(from, to, radius < 5 * voxel_size );

    foreach(Voxel center, path)
    {
        foreach(Voxel cv, cross_section)
        {
            Voxel v = (center) + cv;
            setVoxel( v.x, v.y, v.z );
        }
    }
}

void DynamicVoxel::addCapsule(const Vector3d &from, const Vector3d &to, double radius)
{
    Vector3d direction = (to - from).normalized();

    this->addHemiSphere(from, radius, -direction);
    this->addCylinder(from, to, radius);
    this->addHemiSphere(to, radius, direction);
}

void DynamicVoxel::addPolyLine(const QVector<Vector3d> &points, double radius)
{
    if(points.size() < 2) return;

    // Start cap
    Vector3d startDirection = points.first() - points[1];
    this->addHemiSphere(points.first(), radius, startDirection);

    // Segments in between
    for(int i = 0; i < points.size() - 1; i++)
    {
        this->addCylinder( points[i], points[i + 1], radius );

        if(i + 1 != points.size() - 1)
            this->addSphere(points[i + 1], radius);
    }

    // End cap
    Vector3d endDirection = points.last() - points[ points.size() - 2 ];
    this->addHemiSphere(points.last(), radius, endDirection);
}

void DynamicVoxel::addTorus(const Vector3d &center, double pathRadius, double circleRadius, const Vector3d &direction)
{
    // Recenter
    foreach(Voxel voxel, voxelTorus(pathRadius, circleRadius)){
        Vector3d v(voxel.x, voxel.y, voxel.z);
        v += center / voxel_size;

        // rotate to direction (todo)
        Vector3d d = direction;
        d = d;

        setVoxel( v.x(), v.y(), v.z() );
    }
}

void DynamicVoxel::addBox(const Vector3d & minimum, const Vector3d & maximum)
{
	Vector3d diag = maximum - minimum;

	int stepsX = qMax(diag.x()+voxel_size*2, voxel_size) / voxel_size;
	int stepsY = qMax(diag.y()+voxel_size*2, voxel_size) / voxel_size;
	int stepsZ = qMax(diag.z()+voxel_size*2, voxel_size) / voxel_size;

	Voxel corner (Vector3d((minimum - Vector3d(voxel_size)) / voxel_size));

	for(int x = 0; x <= stepsX; x++){
		for(int y = 0; y <= stepsY; y++){
			for(int z = 0; z <= stepsZ; z++){
				Voxel v = corner + Voxel(x,y,z);
				setVoxel( v.x, v.y, v.z );
			}
		}
	}
}

void DynamicVoxel::begin()
{
    voxels.clear();
}

void DynamicVoxel::setVoxel(int x, int y, int z){
    Voxel v(x,y,z);
    voxels.push_back(v);

    minVoxel.toMin(v);
    maxVoxel.toMax(v);
}

void DynamicVoxel::end()
{
	QElapsedTimer timer; timer.start();

	// Weld
	std::vector<size_t> xrefs;
    weldVoxel(voxels, xrefs, std::hashv_DynamicVoxelLibVoxel(), std::equal_to<Voxel>());

    qDebug() << "Voxel weld operation " << timer.elapsed() << " ms";
}

void DynamicVoxel::buildMesh(SurfaceMesh::Model * mesh)
{
    QuadMesh qmesh;
    buildMesh(mesh, qmesh);
}

void DynamicVoxel::buildMesh(SurfaceMesh::Model * mesh, QuadMesh & m)
{
	QElapsedTimer timer;
	timer.start();

	qDebug() << "Building mesh..";

	m.clear();

	// Add faces
	std::vector<Vector3d> voxelCorners;
	std::vector<Vector3d> allFaceCenters;
	std::vector<QuadFace> allQuads;

	foreach(Voxel v, voxels)
	{
		for(int i = 0; i < 6; i++)
		{
			Vector3d p = (v.toVector3d() + faceCenters[i]) * voxel_size - Vector3d(voxel_size * 0.5);
			allFaceCenters.push_back( p );

			// Add redundant quad face
			QuadFace f;
			for(int j = 0; j < 4; j++)
			{
				f[j] = voxelCorners.size();
				voxelCorners.push_back( (v.toVector3d() + faceCorners[i][j]) * voxel_size - Vector3d(voxel_size * 0.5) );
			}
			allQuads.push_back(f);
		}
	}

	std::vector<size_t> corner_xrefs;
    weldVoxel(voxelCorners, corner_xrefs, std::hashv_Vector3d(), std::equal_to<Vector3d>());

	std::vector<size_t> face_xrefs;
    weldVoxel(allFaceCenters, face_xrefs, std::hashv_Vector3d(), std::equal_to<Vector3d>());
	
	// Count face center occurrences
	std::vector<int> fcount(allFaceCenters.size(), 0);
	for (int i = 0; i < (int)face_xrefs.size(); i++)
		fcount[face_xrefs[i]]++;

	// Find shell faces
	QSet<int> vidxs;
	for(int i = 0; i < (int)face_xrefs.size(); i++)
	{
		int fi = face_xrefs[i];

		// Shells only exist once
		if(fcount[fi] == 2) continue; 
		
		// Assign vertex to its unique id + add to a SET
		for(int j = 0; j < 4; j++) 
		{
			allQuads[i][j] = corner_xrefs[allQuads[i][j]];
			vidxs.insert(allQuads[i][j]);
		}

 		m.faces.push_back(allQuads[i]);
	}

	// Map to ordered indices
	std::map<int,int> vmap;
	foreach(int vi, vidxs){
		vmap[vi] = vmap.size();
		m.points.push_back( voxelCorners[vi] );
	}

	// Replace vertex index to ordered index
	for(int i = 0; i < (int)m.faces.size(); i++)
	{
		for(int j = 0; j < 4; j++) 
			m.faces[i][j] = vmap[m.faces[i][j]];
	}

	qDebug() << "Found shell faces: " << timer.elapsed();
	timer.restart();

	// Create a Surface_mesh
	if(mesh)
	{
		// Add vertices
		foreach(Vector3 p, m.points)
		{
			mesh->add_vertex(p);
		}

		// Add faces
		foreach(QuadFace f, m.faces)
		{
			std::vector<Vertex> faceVerts;
			for(int i = 0; i < 4; i++) 
			{
				faceVerts.push_back( Vertex( f[i] ) );
			}
			mesh->add_face( faceVerts );
		}

        // Handle non-manifold voxel configurations: very simplistic
        if(m.faces.size() != mesh->n_faces())
        {
			int counter = 0;

			while(hasHoles(mesh))
			{
				FillHoles(mesh, voxel_size);
			
				if(counter++ > 5) break;
			}
        }

		qDebug() << "Final mesh creation: " << timer.elapsed();
	}
}

void DynamicVoxel::MeanCurvatureFlow(SurfaceMesh::Model * m, double dt)
{
	QElapsedTimer timer; timer.start();

    int N = m->n_vertices();

    VectorXd bx(N), by(N), bz(N);
    VectorXd X(N), Y(N), Z(N);

    Surface_mesh::Vertex_property<Point> points = m->vertex_property<Point>("v:point");
    Surface_mesh::Face_around_vertex_circulator fvit, fvend;
    Surface_mesh::Halfedge_around_vertex_circulator h, hend;

    SurfaceMeshHelper helper(m);

    ScalarFaceProperty farea = helper.computeFaceAreas();
    std::vector<double> area(N, 0.0);

    // Efficient sparse matrix construction
    typedef Eigen::Triplet<double> T;
    std::vector< T > L;

    double cots, cot_alpha, cot_beta;

    // For each point
    for(int i = 0; i < N; i++)
    {
        Surface_mesh::Vertex v(i);

        // for each neighboring face
        fvit = fvend = m->faces(v);
        do{ area[i] += farea[fvit]; } while (++fvit != fvend);

        L.push_back(T(i,i,area[i]));

        // for each neighboring vertex
        if(!m->is_boundary(v))
        {
            Point p0 = points[v];

            h = m->halfedges(v), hend = m->halfedges(v);
            do{
                Surface_mesh::Vertex vj = m->to_vertex(h);
                int j = vj.idx();

                Point p1 = points[vj];
                Point p2 = points[m->to_vertex(m->next_halfedge(h))];
                Point p3 = points[m->to_vertex(m->next_halfedge(m->opposite_halfedge(h)))];

                double cos_alpha = dot((p0 - p2).normalized(), (p1 - p2).normalized());
                double cos_beta = dot((p0 - p3).normalized(), (p1 - p3).normalized());

                cot_alpha = cos_alpha / sin(acos( qRanged(-1.0, cos_alpha, 1.0) ));
                cot_beta = cos_beta / sin(acos( qRanged(-1.0, cos_beta, 1.0) ));

                cots = (cot_alpha + cot_beta) * 0.25 * dt;

                //if(cot_alpha == 0 || cot_beta == 0)
                //    cots = 0;

                if(i > j)
                {
                    L.push_back(T(i,j, -cots));
                    L.push_back(T(j,i, -cots));
                }

                L.push_back(T(i,i, cots));

            } while( ++h != hend );
        }

        X(i) = points[v].x()   ; Y(i) = points[v].y()    ; Z(i) = points[v].z();
        bx(i) = area[i] * X(i) ; by(i) = area[i] * Y(i)  ; bz(i) = area[i] * Z(i);
    }

    SparseMatrix<Scalar> L_mat(m->n_vertices(), m->n_vertices());
    L_mat.setFromTriplets(L.begin(), L.end());

    CholmodSolver solver(L_mat);

    X = solver.solve(bx);
    Y = solver.solve(by);
    Z = solver.solve(bz);

	// Set to new positions
    for(int i = 0; i < N; i++)
        points[Surface_mesh::Vertex(i)] = Point(X(i), Y(i), Z(i));

	qDebug() << "MCF smoothing " << timer.elapsed() << " ms";
}

void DynamicVoxel::LaplacianSmoothing(SurfaceMesh::Model * m, bool protectBorders)
{
    Surface_mesh::Vertex_iterator vit, vend = m->vertices_end();
    Surface_mesh::Vertex_around_vertex_circulator vvit, vvend;

    Surface_mesh::Vertex_property<Point> points = m->vertex_property<Point>("v:point");
    Surface_mesh::Vertex_property<Point> newPositions = m->vertex_property<Point>("v:new_point", Vector3(0,0,0));

    // Original positions
    for(vit = m->vertices_begin(); vit != vend; ++vit)
        newPositions[vit] = points[vit];

    // Compute Laplacian
    for(vit = m->vertices_begin(); vit != vend; ++vit)
    {
        if(!protectBorders || (protectBorders && !m->is_boundary(vit)))
        {
            newPositions[vit] = Point(0,0,0);

            // Sum up neighbors
            vvit = vvend = m->vertices(vit);
            do{ newPositions[vit] += points[vvit]; } while(++vvit != vvend);

            // Average it
            newPositions[vit] /= m->valence(vit);
        }
    }

    // Set vertices to final position
    for(vit = m->vertices_begin(); vit != vend; ++vit)
        points[vit] = newPositions[vit];

    m->remove_vertex_property(newPositions);
}

void DynamicVoxel::FillHoles( SurfaceMesh::Model *mesh, double voxel_length )
{
	Vector3VertexProperty mesh_points = mesh->vertex_property<Vector3>(VPOINT);

	// Find holes
	QList<Halfedge> all_holes;
	QVector<Halfedge> holes;

	foreach(Halfedge h, mesh->halfedges())
		if(mesh->is_boundary(h)) all_holes.push_back(h);

	// Find representative halfedge for each hole
	while(!all_holes.empty()){
		Halfedge h = all_holes.first(), end = h;
		all_holes.removeFirst();
		holes.push_back(h);
		do{
			h = mesh->next_halfedge(h);
			all_holes.removeAll(h);
		} while (h != end);
	}

	// Find a good center for each hole
	QVector<Vector3> hole_center;
	foreach(Halfedge h, holes)
	{
		Halfedge end = h;

		Vector3 sum(0,0,0);
		std::vector<Vector3> pnts;
		int c = 0;

		do{ 
			pnts.push_back(mesh_points[ mesh->to_vertex(h) ]); 
			sum += pnts.back(); 
			c++; 
			h = mesh->next_halfedge(h); 
		} while (h != end);

		// Give it a nudge
		Vector3 nudge(0,0,0);
		findNormal3D(pnts, nudge);
		nudge = nudge.normalized() * 0.5 * sqrt( voxel_length*voxel_length + voxel_length*voxel_length );

		hole_center.push_back((sum / c) + nudge);
	}

	// Hole filling structure
	QVector< std::vector<Vertex> > filler;
	for(int i = 0; i < (int)holes.size(); i++)
	{
		Halfedge h = holes[i], end = h;
		Vertex c = mesh->add_vertex( hole_center[i] );

		do{ 
			Vertex a = mesh->to_vertex(h);
			Vertex b = mesh->from_vertex(h);
			h = mesh->next_halfedge(h); 

			std::vector<Vertex> triangle;

			triangle.push_back(b);
			triangle.push_back(a);
			triangle.push_back(c);

			filler.push_back(triangle);
		} while (h != end);
	}

	// Close the holes
	foreach(std::vector<Vertex> triangle, filler)
		mesh->add_face(triangle);

	// Clean up
	foreach(Vertex v, mesh->vertices())	if(mesh->is_isolated(v)) mesh->remove_vertex(v);
	mesh->garbage_collection();
}

bool DynamicVoxel::hasHoles( SurfaceMesh::Model *m )
{
	foreach(Edge e, m->edges())
		if(m->is_boundary(e))
			return true;

	return false;
}
