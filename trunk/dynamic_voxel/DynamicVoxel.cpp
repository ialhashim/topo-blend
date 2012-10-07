#include "StarlabDrawArea.h"
#include "DynamicVoxel.h"
#include "SurfaceMeshHelper.h"
#include <QElapsedTimer>

DynamicVoxel::DynamicVoxel(double voxelSize){

    this->voxel_size = voxelSize;

    this->maxVoxel = Voxel(-DBL_MAX);
    this->minVoxel = Voxel(DBL_MAX);

    //this->addSphere(Vector3(0,0,0), 1);
    //this->addCircle(Vector3(0), 1, Vec3d(1,1,1));
    Vec3d to(2,2,0);
    double radius = 1;
    //this->addLine(Vector3(0), to);
    //this->addCircle(Vector3(0), radius, (to - Vector3(0)).normalized());
    //this->addCapsule(Vector3(0), to, radius);

    /*setVoxel(0,0,0);
    setVoxel(1,0,0);
    setVoxel(1,1,0);
    setVoxel(1,0,1);
    setVoxel(-1,0,0);
    setVoxel(-1,-1,0);
    setVoxel(-1,1,0);
    setVoxel(1,-1,0);*/
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

                if(r <= radius && r >= (radius - voxel_size*2)){
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

std::vector<Voxel> DynamicVoxel::orientedVoxelCircle(double radius, const Vec3d &direction)
{
    std::vector<Voxel> result;
    std::vector<Voxel> sphere = voxelSphere(radius*2);

    foreach(Voxel voxel, sphere){
        Vec3d v(voxel.x, voxel.y, voxel.z);
		v /= 2.0;

        if(abs(dot(direction, v.normalized())) < voxel_size)
            result.push_back(Voxel(v));
    }

    return result;
}

std::vector<Voxel> DynamicVoxel::voxelLine(const Vec3d &p1, const Vec3d &p2, bool thick)
{
    std::vector<Voxel> line;

    Vec3d d = (p2 - p1) / voxel_size;
    double N = qMax( abs(d.x()), qMax(abs(d.y()), abs(d.z())) );
    Vec3d s = d / N;

    Vec3d p = p1;
    line.push_back( Voxel(p) );
    for(int i = 0; i < N; i++){
        p = p + s;
        line.push_back( Voxel(p) );

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

void DynamicVoxel::addLine(const Vec3d &p1, const Vec3d &p2)
{
    std::vector<Voxel> line = voxelLine(p1, p2);

    foreach(Voxel v, line)
        setVoxel( v.x, v.y, v.z );
}

void DynamicVoxel::addCircle(const Vec3d &center, double radius, const Vec3d &direction)
{
    std::vector<Voxel> circle3D = orientedVoxelCircle(radius, direction);

    foreach(Voxel voxel, circle3D){
        Vec3d v(voxel.x, voxel.y, voxel.z);
        v += center / voxel_size;
        setVoxel( v.x(), v.y(), v.z() );
    }
}

void DynamicVoxel::addSphere(const Vec3d &center, double radius)
{
    // Recenter
    foreach(Voxel voxel, voxelSphere(radius)){
        Vec3i v(voxel.x, voxel.y, voxel.z);
        v += center / voxel_size;
        setVoxel( v.x(), v.y(), v.z() );
    }
}

void DynamicVoxel::addHemiSphere(const Vec3d &center, double radius, const Vec3d & direction)
{
    std::vector<Voxel> hemisphere = voxelSphere(radius);

    // Recenter
    foreach(Voxel voxel, hemisphere){
        Vec3d v(voxel.x, voxel.y, voxel.z);

        if(dot(direction, v) > 0)
        {
            v += center / voxel_size;
            setVoxel( v.x(), v.y(), v.z() );
        }
    }
}

void DynamicVoxel::addCylinder(const Vec3d &from, const Vec3d &to, double radius){
    Vec3d direction = (from - to).normalized();

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

void DynamicVoxel::addCapsule(const Vec3d &from, const Vec3d &to, double radius)
{
    QElapsedTimer timer;
    timer.start();

    this->addHemiSphere(from, radius, -to.normalized());
    this->addCylinder(from, to, radius);
    this->addHemiSphere(to, radius, to.normalized());

    qDebug() << "Voxel creation: " << timer.elapsed();
}

void DynamicVoxel::setVoxel(int x, int y, int z){
    Voxel v(x,y,z);
    voxels.insert( v );

    minVoxel.toMin(v);
    maxVoxel.toMax(v);
}

void DynamicVoxel::buildMesh(SurfaceMeshModel * m)
{
    QElapsedTimer timer;
    timer.start();

    NanoKdTree corner_tree, face_tree;
    std::vector<QuadFace> faces;

    // Add all corners
    foreach(Voxel v, voxels)
        for(int i = 0; i < 8; i++)
            corner_tree.addPoint((v + cornerVoxels[i]) * voxel_size);

    corner_tree.build();

    // Add all faces
    foreach(Voxel v, voxels){
        for(int i = 0; i < 6; i++){
            Vec3d p = (v.toVec3d() + faceCenters[i]) * voxel_size;
            face_tree.addPoint(p);
        }
    }

    face_tree.build();

    qDebug() << "KD trees: " << timer.elapsed();
    timer.restart();

    // Find shell faces
    foreach(Voxel v, voxels){
        for(int i = 0; i < 6; i++)
        {
            Vec3d p = (v.toVec3d() + faceCenters[i]) * voxel_size;

            // A unique outer face only exists once
            if(face_tree.ball_search(p, voxel_size * 0.25) > 1) continue;

            QuadFace f;

            for(int j = 0; j < 4; j++)
            {
                Vector3 corner = (v.toVec3d() + faceCorners[i][j]) * voxel_size;
                f[j] = corner_tree.closest(corner);
            }

            faces.push_back(f);
        }
    }

    qDebug() << "Find shell faces: " << timer.elapsed();
    timer.restart();

    // Collect set of vertices
    QSet<int> verts;
    foreach(QuadFace f, faces)
        for(int i = 0; i < 4; i++)
            verts.insert(f[i]);

    // Map to ordered indices & add vertices to mesh
    std::map<int,int> vmap;
    foreach(int vi, verts){
        vmap[vi] = vmap.size();
        m->add_vertex(Vector3( corner_tree.cloud.pts[vi] ));
    }

    qDebug() << "Mapping: " << timer.elapsed();
    timer.restart();

    // Add faces
    foreach(QuadFace f, faces)
        m->add_quad( Vertex(vmap[f[0]]), Vertex(vmap[f[1]]), Vertex(vmap[f[2]]), Vertex(vmap[f[3]]) );

    qDebug() << "Mesh creation: " << timer.elapsed();
    timer.restart();

}
