#pragma once
#include <qgl.h>

namespace DynamicVoxelLib{

struct Voxel{
    int x, y, z;
    int flag;

    Voxel(){ x = y = z = flag = 0; }
    Voxel(int X, int Y, int Z = 0) : x(X), y(Y), z(Z){ flag = 1; }
    Voxel(const Vec3i & p) { x = p.x(); y = p.y(); z = p.z(); }

    // Operators
    operator const Vec3d() const{ return Vec3d(x,y,z); }
    inline Vec3d toVec3d() const{ return Vec3d(x,y,z); }

	const Voxel & operator =(const Voxel & v) { 
		x = v.x; y = v.y; z = v.z; return *this; 
	}

    bool operator== (const Voxel & other) const{
        return this->x == other.x && this->y == other.y && this->z == other.z;
    }

	bool operator!=(const Voxel & v) const { 
		return x != v.x || y != v.y || z != v.z; 
	}

    Voxel & operator+= (const Voxel & other){
        x += other.x;   y += other.y;   z += other.z;
        return *this;
    }

    Voxel & operator-= (const Voxel & other){
        x -= other.x;   y -= other.y;   z -= other.z;
        return *this;
    }

    Voxel operator+(const Voxel & other) const{
        return Voxel(*this) += other;
    }

    Voxel operator-(const Voxel & other) const{
        return Voxel(*this) -= other;
    }

    Vec3d operator/(const double & val) const{
        return Vec3d(x / val, y / val, z / val);
    }

	Vec3d operator*(const double & val) const{
		return Vec3d(x * val, y * val, z * val);
	}

    // Useful for bounds
    inline void toMax(const Voxel & v){ x = qMax(x, v.x); y = qMax(y, v.y); z = qMax(z, v.z); }
    inline void toMin(const Voxel & v){ x = qMin(x, v.x); y = qMin(y, v.y); z = qMin(z, v.z); }
};

static inline QDebug operator<< (QDebug d, const Voxel &v) {
    d << v.x << " | " << v.y << " | " << v.z;
    return d;
}

static Voxel cornerVoxels[] = { Voxel(0,0,0), Voxel(1,0,0), Voxel(0,1,0), Voxel(1,1,0),
                                Voxel(0,0,1), Voxel(1,0,1), Voxel(0,1,1), Voxel(1,1,1)};

static Vec3d faceCenters[] = { Vec3d(0.5,0.5,0),
                               Vec3d(0.5,0,0.5), Vec3d(0,0.5,0.5), Vec3d(1,0.5,0.5), Vec3d(0.5,1,0.5),
                               Vec3d(0.5,0.5,1)};

static Voxel faceCentersVoxel[] = { Voxel(0,0,-1),
									Voxel(0,-1,0), Voxel(-1,0,0), Voxel(1,0,0), Voxel(0,1,0),
									Voxel(0,0,1)};

static Vec3d faceCorners[6][4] = { { Vec3d(0,0,0), Vec3d(0,1,0), Vec3d(1,1,0), Vec3d(1,0,0) },
                                   { Vec3d(0,0,0), Vec3d(1,0,0), Vec3d(1,0,1), Vec3d(0,0,1) },
                                   { Vec3d(0,0,0), Vec3d(0,0,1), Vec3d(0,1,1), Vec3d(0,1,0) },
                                   { Vec3d(1,0,0), Vec3d(1,1,0), Vec3d(1,1,1), Vec3d(1,0,1) },
                                   { Vec3d(0,1,0), Vec3d(0,1,1), Vec3d(1,1,1), Vec3d(1,1,0) },
                                   { Vec3d(0,0,1), Vec3d(1,0,1), Vec3d(1,1,1), Vec3d(0,1,1) }};

static Voxel faceCornersVoxel[6][4] = { { Voxel(0,0,0), Voxel(0,1,0), Voxel(1,1,0), Voxel(1,0,0) },
										{ Voxel(0,0,0), Voxel(1,0,0), Voxel(1,0,1), Voxel(0,0,1) },
										{ Voxel(0,0,0), Voxel(0,0,1), Voxel(0,1,1), Voxel(0,1,0) },
										{ Voxel(1,0,0), Voxel(1,1,0), Voxel(1,1,1), Voxel(1,0,1) },
										{ Voxel(0,1,0), Voxel(0,1,1), Voxel(1,1,1), Voxel(1,1,0) },
										{ Voxel(0,0,1), Voxel(1,0,1), Voxel(1,1,1), Voxel(0,1,1) }};

struct QuadFace{
    int v[4];
    QuadFace(){ v[0] = v[1] = v[2] = v[3] = -1; }
    QuadFace(int v0, int v1, int v2, int v3){
        v[0] = v0; v[1] = v1;
        v[2] = v2; v[3] = v3;
    }
    int& operator[](unsigned int i){ return v[i];}
};

static inline uint qHash( const Voxel &key ){return qHash( QString("%1%2%3").arg(key.x).arg(key.y).arg(key.z) ); }

static void drawCube(double x, double y, double z, double scale = 1.0){

    glPushMatrix();

    glScaled(scale, scale, scale);
    glTranslated(x, y, z);

    static GLdouble n[6][3] ={
        {-0.1,  0.0,  0.0},
        { 0.0,  0.1,  0.0},
        { 0.1,  0.0,  0.0},
        { 0.0, -0.1,  0.0},
        { 0.0,  0.0,  0.1},
        { 0.0,  0.0, -0.1}};

    static GLint faces[6][4] ={
        {0, 1, 2, 3},
        {3, 2, 6, 7},
        {7, 6, 5, 4},
        {4, 5, 1, 0},
        {5, 6, 2, 1},
        {7, 4, 0, 3}};

    GLdouble v[8][3];GLint i;

    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -0.5;
    v[4][0] = v[5][0] = v[6][0] = v[7][0] =  0.5;
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -0.5;
    v[2][1] = v[3][1] = v[6][1] = v[7][1] =  0.5;
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = -0.5;
    v[1][2] = v[2][2] = v[5][2] = v[6][2] =  0.5;

    glBegin(GL_QUADS);
    for (i = 0; i < 6; i++)
    {
        glNormal3dv(&n[i][0]);
        glVertex3dv(&v[faces[i][0]][0]);
        glVertex3dv(&v[faces[i][1]][0]);
        glVertex3dv(&v[faces[i][2]][0]);
        glVertex3dv(&v[faces[i][3]][0]);
    }
    glEnd();

    glPopMatrix();
}

}

namespace std {
    struct hashv_Vec3d {
        size_t operator()(Vec3d v) {
            const unsigned int * h = (const unsigned int *)(&v);
            unsigned int f = (h[0]+h[1]*11-(h[2]*17))&0x7fffffff;     // avoid problems with +-0
            return (f>>22)^(f>>12)^(f);
        }
    };
    struct hashv_Vec3f {
        size_t operator()(Vec3f v) {
            const unsigned int * h = (const unsigned int *)(&v);
            unsigned int f = (h[0]+h[1]*11-(h[2]*17))&0x7fffffff;     // avoid problems with +-0
            return (f>>22)^(f>>12)^(f);
        }
    };
    struct hashv_DynamicVoxelLibVoxel {
        size_t operator()(DynamicVoxelLib::Voxel v) {
			const unsigned int * h = (const unsigned int *)(&v);
			unsigned int f = (h[0]+h[1]*11-(h[2]*17))&0x7fffffff;     // avoid problems with +-0
			return (f>>22)^(f>>12)^(f);
		}
	};
}

#define qRanged(min, v, max) ( qMax(min, qMin(v, max)) )
