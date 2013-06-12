#pragma once

typedef Eigen::Vector2d Vector2;

// Rotation matrices are of the form
//   R = cos(t) -sin(t)
//       sin(t)  cos(t)
// where t > 0 indicates a counterclockwise rotation in the xy-plane.

template <typename Real>
class Matrix2
{
public:
    // If makeZero is 'true', create the zero matrix; otherwise, create the
    // identity matrix.
    Matrix2 (bool makeZero = true);

    // Copy constructor.
    Matrix2 (const Matrix2& mat);

    // Input mrc is in row r, column c.
    Matrix2 (Real m00, Real m01, Real m10, Real m11);

    // Create a matrix from an array of numbers.  The input array is
    // interpreted based on the bool input as
    //   true:  entry[0..3] = {m00,m01,m10,m11}  [row major]
    //   false: entry[0..3] = {m00,m10,m01,m11}  [column major]
    Matrix2 (const Real entry[4], bool rowMajor);

    // Create matrices based on vector input.  The bool is interpreted as
    //   true: vectors are columns of the matrix
    //   false: vectors are rows of the matrix
    Matrix2 (const Vector2& u, const Vector2& v, bool columns);
    Matrix2 (const Vector2* vectors, bool columns);

    // Create a diagonal matrix, m01 = m10 = 0.
    Matrix2 (Real m00, Real m11);

    // Create a rotation matrix (positive angle -> counterclockwise).
    Matrix2 (Real angle);

    // Create a tensor product U*V^T.
    Matrix2 (const Vector2& u, const Vector2& v);

    // Assignment.
    Matrix2& operator= (const Matrix2& mat);

    // Create various matrices.
    void MakeZero ();
    void MakeIdentity ();
    void MakeDiagonal (Real m00, Real m11);
    void MakeRotation (Real angle);
    void MakeTensorProduct (const Vector2& u, const Vector2& v);

    // Arithmetic operations.
    Matrix2 operator+ (const Matrix2& mat) const;
    Matrix2 operator- (const Matrix2& mat) const;
    Matrix2 operator* (Real scalar) const;
    Matrix2 operator/ (Real scalar) const;
    Matrix2 operator- () const;

    // Arithmetic updates.
    Matrix2& operator+= (const Matrix2& mat);
    Matrix2& operator-= (const Matrix2& mat);
    Matrix2& operator*= (Real scalar);
    Matrix2& operator/= (Real scalar);

    // M*vec
    Vector2 operator* (const Vector2& vec) const;

    // u^T*M*v
    Real QForm (const Vector2& u, const Vector2& v) const;

    // M^T
    Matrix2 Transpose () const;

    // M*mat
    Matrix2 operator* (const Matrix2& mat) const;

    // M^T*mat
    Matrix2 TransposeTimes (const Matrix2& mat) const;

    // M*mat^T
    Matrix2 TimesTranspose (const Matrix2& mat) const;

    // M^T*mat^T
    Matrix2 TransposeTimesTranspose (const Matrix2& mat) const;

    // Other operations.
    Matrix2 Inverse (const Real epsilon = (Real)0) const;
    Matrix2 Adjoint () const;
    Real Determinant () const;

    // The matrix must be a rotation for these functions to be valid.  The
    // last function uses Gram-Schmidt orthonormalization applied to the
    // columns of the rotation matrix.  The angle must be in radians, not
    // degrees.
    void ExtractAngle (Real& angle) const;
    void Orthonormalize ();

    // The matrix must be symmetric.  Factor M = R * D * R^T where
    // R = [u0|u1] is a rotation matrix with columns u0 and u1 and
    // D = diag(d0,d1) is a diagonal matrix whose diagonal entries are d0
    // and d1.  The eigenvector u[i] corresponds to eigenvector d[i].  The
    // eigenvalues are ordered as d0 <= d1.
    void EigenDecomposition (Matrix2& rot, Matrix2& diag) const;

    // Special matrices.
    static const Matrix2 ZERO;
    static const Matrix2 IDENTITY;

	inline const Real* operator[] (int row) const{ return &mEntry[2*row]; }
	inline Real* operator[] (int row){ return &mEntry[2*row]; }

protected:
	Real mEntry[4];
};

// c * M
template <typename Real>
inline Matrix2<Real> operator* (Real scalar, const Matrix2<Real>& mat);

// v^T * M
template <typename Real>
inline Vector2 operator* (const Vector2& vec, const Matrix2<Real>& mat);


//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (bool makeZero)
{
    if (makeZero)
    {
        MakeZero();
    }
    else
    {
        MakeIdentity();
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (const Matrix2& mat)
{
    mEntry[0] = mat.mEntry[0];
    mEntry[1] = mat.mEntry[1];
    mEntry[2] = mat.mEntry[2];
    mEntry[3] = mat.mEntry[3];
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (Real m00, Real m01, Real m10, Real m11)
{
    mEntry[0] = m00;
    mEntry[1] = m01;
    mEntry[2] = m10;
    mEntry[3] = m11;
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (const Real entry[4], bool rowMajor)
{
    if (rowMajor)
    {
        mEntry[0] = entry[0];
        mEntry[1] = entry[1];
        mEntry[2] = entry[2];
        mEntry[3] = entry[3];
    }
    else
    {
        mEntry[0] = entry[0];
        mEntry[1] = entry[2];
        mEntry[2] = entry[1];
        mEntry[3] = entry[3];
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (const Vector2& u, const Vector2& v,
    bool columns)
{
    if (columns)
    {
        mEntry[0] = u[0];
        mEntry[1] = v[0];
        mEntry[2] = u[1];
        mEntry[3] = v[1];
    }
    else
    {
        mEntry[0] = u[0];
        mEntry[1] = u[1];
        mEntry[2] = v[0];
        mEntry[3] = v[1];
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (const Vector2* v, bool columns)
{
    if (columns)
    {
        mEntry[0] = v[0][0];
        mEntry[1] = v[1][0];
        mEntry[2] = v[0][1];
        mEntry[3] = v[1][1];
    }
    else
    {
        mEntry[0] = v[0][0];
        mEntry[1] = v[0][1];
        mEntry[2] = v[1][0];
        mEntry[3] = v[1][1];
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (Real m00, Real m11)
{
    MakeDiagonal(m00, m11);
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (Real angle)
{
    MakeRotation(angle);
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>::Matrix2 (const Vector2& u, const Vector2& v)
{
    MakeTensorProduct(u, v);
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>& Matrix2<Real>::operator= (const Matrix2& mat)
{
    mEntry[0] = mat.mEntry[0];
    mEntry[1] = mat.mEntry[1];
    mEntry[2] = mat.mEntry[2];
    mEntry[3] = mat.mEntry[3];
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
void Matrix2<Real>::MakeZero ()
{
    mEntry[0] = (Real)0;
    mEntry[1] = (Real)0;
    mEntry[2] = (Real)0;
    mEntry[3] = (Real)0;
}
//----------------------------------------------------------------------------
template <typename Real>
void Matrix2<Real>::MakeIdentity ()
{
    mEntry[0] = (Real)1;
    mEntry[1] = (Real)0;
    mEntry[2] = (Real)0;
    mEntry[3] = (Real)1;
}
//----------------------------------------------------------------------------
template <typename Real>
void Matrix2<Real>::MakeDiagonal (Real m00, Real m11)
{
    mEntry[0] = m00;
    mEntry[1] = (Real)0;
    mEntry[2] = (Real)0;
    mEntry[3] = m11;
}
//----------------------------------------------------------------------------
template <typename Real>
void Matrix2<Real>::MakeRotation (Real angle)
{
    mEntry[0] = cos(angle);
    mEntry[2] = sin(angle);
    mEntry[1] = -mEntry[2];
    mEntry[3] =  mEntry[0];
}
//----------------------------------------------------------------------------
template <typename Real>
void Matrix2<Real>::MakeTensorProduct (const Vector2& u,
    const Vector2& v)
{
    mEntry[0] = u[0]*v[0];
    mEntry[1] = u[0]*v[1];
    mEntry[2] = u[1]*v[0];
    mEntry[3] = u[1]*v[1];
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::operator+ (const Matrix2& mat) const
{
    return Matrix2<Real>
    (
        mEntry[0] + mat.mEntry[0],
        mEntry[1] + mat.mEntry[1],
        mEntry[2] + mat.mEntry[2],
        mEntry[3] + mat.mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::operator- (const Matrix2& mat)
    const
{
    return Matrix2<Real>
    (
        mEntry[0] - mat.mEntry[0],
        mEntry[1] - mat.mEntry[1],
        mEntry[2] - mat.mEntry[2],
        mEntry[3] - mat.mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::operator* (Real scalar) const
{
    return Matrix2<Real>
    (
        scalar*mEntry[0],
        scalar*mEntry[1],
        scalar*mEntry[2],
        scalar*mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::operator/ (Real scalar) const
{
    if (scalar != (Real)0)
    {
        Real invScalar = ((Real)1)/scalar;
        return Matrix2<Real>
        (
            invScalar*mEntry[0],
            invScalar*mEntry[1],
            invScalar*mEntry[2],
            invScalar*mEntry[3]
        );
    }
    else
    {
        return Matrix2<Real>
        (
            DBL_MAX,
            DBL_MAX,
            DBL_MAX,
            DBL_MAX
        );
    }
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::operator- () const
{
    return Matrix2<Real>
    (
        -mEntry[0],
        -mEntry[1],
        -mEntry[2],
        -mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>& Matrix2<Real>::operator+= (const Matrix2& mat)
{
    mEntry[0] += mat.mEntry[0];
    mEntry[1] += mat.mEntry[1];
    mEntry[2] += mat.mEntry[2];
    mEntry[3] += mat.mEntry[3];
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>& Matrix2<Real>::operator-= (const Matrix2& mat)
{
    mEntry[0] -= mat.mEntry[0];
    mEntry[1] -= mat.mEntry[1];
    mEntry[2] -= mat.mEntry[2];
    mEntry[3] -= mat.mEntry[3];
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>& Matrix2<Real>::operator*= (Real scalar)
{
    mEntry[0] *= scalar;
    mEntry[1] *= scalar;
    mEntry[2] *= scalar;
    mEntry[3] *= scalar;
    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real>& Matrix2<Real>::operator/= (Real scalar)
{
    if (scalar != (Real)0)
    {
        Real invScalar = ((Real)1)/scalar;
        mEntry[0] *= invScalar;
        mEntry[1] *= invScalar;
        mEntry[2] *= invScalar;
        mEntry[3] *= invScalar;
    }
    else
    {
        mEntry[0] = DBL_MAX;
        mEntry[1] = DBL_MAX;
        mEntry[2] = DBL_MAX;
        mEntry[3] = DBL_MAX;
    }

    return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
Vector2 Matrix2<Real>::operator* (const Vector2& vec) const
{
    return Vector2
    (
        mEntry[0]*vec[0] + mEntry[1]*vec[1],
        mEntry[2]*vec[0] + mEntry[3]*vec[1]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Real Matrix2<Real>::QForm (const Vector2& u, const Vector2& v)
    const
{
    return dot(u,(*this)*v);
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::Transpose () const
{
    return Matrix2<Real>
    (
        mEntry[0],
        mEntry[2],
        mEntry[1],
        mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::operator* (const Matrix2& mat) const
{
    // A*B
    return Matrix2<Real>
    (
        mEntry[0]*mat.mEntry[0] + mEntry[1]*mat.mEntry[2],
        mEntry[0]*mat.mEntry[1] + mEntry[1]*mat.mEntry[3],
        mEntry[2]*mat.mEntry[0] + mEntry[3]*mat.mEntry[2],
        mEntry[2]*mat.mEntry[1] + mEntry[3]*mat.mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::TransposeTimes (const Matrix2& mat) const
{
    // A^T*B
    return Matrix2<Real>
    (
        mEntry[0]*mat.mEntry[0] + mEntry[2]*mat.mEntry[2],
        mEntry[0]*mat.mEntry[1] + mEntry[2]*mat.mEntry[3],
        mEntry[1]*mat.mEntry[0] + mEntry[3]*mat.mEntry[2],
        mEntry[1]*mat.mEntry[1] + mEntry[3]*mat.mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::TimesTranspose (const Matrix2& mat) const
{
    // A*B^T
    return Matrix2<Real>
    (
        mEntry[0]*mat.mEntry[0] + mEntry[1]*mat.mEntry[1],
        mEntry[0]*mat.mEntry[2] + mEntry[1]*mat.mEntry[3],
        mEntry[2]*mat.mEntry[0] + mEntry[3]*mat.mEntry[1],
        mEntry[2]*mat.mEntry[2] + mEntry[3]*mat.mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::TransposeTimesTranspose (const Matrix2& mat)
    const
{
    // A^T*B^T
    return Matrix2<Real>
    (
        mEntry[0]*mat.mEntry[0] + mEntry[2]*mat.mEntry[1],
        mEntry[0]*mat.mEntry[2] + mEntry[2]*mat.mEntry[3],
        mEntry[1]*mat.mEntry[0] + mEntry[3]*mat.mEntry[1],
        mEntry[1]*mat.mEntry[2] + mEntry[3]*mat.mEntry[3]
    );
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::Inverse (const Real epsilon) const
{
    Matrix2<Real> inverse;

    Real det = mEntry[0]*mEntry[3] - mEntry[1]*mEntry[2];
    if (fabs(det) > epsilon)
    {
        Real invDet = ((Real)1)/det;
        inverse.mEntry[0] =  mEntry[3]*invDet;
        inverse.mEntry[1] = -mEntry[1]*invDet;
        inverse.mEntry[2] = -mEntry[2]*invDet;
        inverse.mEntry[3] =  mEntry[0]*invDet;
    }
    else
    {
        inverse.mEntry[0] = (Real)0;
        inverse.mEntry[1] = (Real)0;
        inverse.mEntry[2] = (Real)0;
        inverse.mEntry[3] = (Real)0;
    }

    return inverse;
}
//----------------------------------------------------------------------------
template <typename Real>
Matrix2<Real> Matrix2<Real>::Adjoint () const
{
    return Matrix2<Real>(mEntry[3], -mEntry[1], -mEntry[2], mEntry[0]);
}
//----------------------------------------------------------------------------
template <typename Real>
Real Matrix2<Real>::Determinant () const
{
    return mEntry[0]*mEntry[3] - mEntry[1]*mEntry[2];
}
//----------------------------------------------------------------------------
template <typename Real>
void Matrix2<Real>::ExtractAngle (Real& angle) const
{
    // assert:  'this' matrix represents a rotation
    angle = atan2(mEntry[2], mEntry[0]);
}
//----------------------------------------------------------------------------
template <typename Real>
void Matrix2<Real>::Orthonormalize ()
{
    // Algorithm uses Gram-Schmidt orthogonalization.  If 'this' matrix is
    // M = [m0|m1], then orthonormal output matrix is Q = [q0|q1],
    //
    //   q0 = m0/|m0|
    //   q1 = (m1-(q0*m1)q0)/|m1-(q0*m1)q0|
    //
    // where |V| indicates length of vector V and A*B indicates dot
    // product of vectors A and B.

    // Compute q0.
    Real invLength = 1.0/sqrt(mEntry[0]*mEntry[0] +
        mEntry[2]*mEntry[2]);

    mEntry[0] *= invLength;
    mEntry[2] *= invLength;

    // Compute q1.
    Real dot0 = mEntry[0]*mEntry[1] + mEntry[2]*mEntry[3];
    mEntry[1] -= dot0*mEntry[0];
    mEntry[3] -= dot0*mEntry[2];

    invLength = 1.0/sqrt(mEntry[1]*mEntry[1] +
        mEntry[3]*mEntry[3]);

    mEntry[1] *= invLength;
    mEntry[3] *= invLength;
}
//----------------------------------------------------------------------------
template <typename Real>
void Matrix2<Real>::EigenDecomposition (Matrix2& rot, Matrix2& diag) const
{
    Real sum = fabs(mEntry[0]) + fabs(mEntry[3]);
    if (fabs(mEntry[1]) + sum == sum)
    {
        // The matrix M is diagonal (within numerical round-off).
        rot.mEntry[0] = (Real)1;
        rot.mEntry[1] = (Real)0;
        rot.mEntry[2] = (Real)0;
        rot.mEntry[3] = (Real)1;
        diag.mEntry[0] = mEntry[0];
        diag.mEntry[1] = (Real)0;
        diag.mEntry[2] = (Real)0;
        diag.mEntry[3] = mEntry[3];
        return;
    }

    Real trace = mEntry[0] + mEntry[3];
    Real diff = mEntry[0] - mEntry[3];
    Real discr = sqrt(diff*diff + ((Real)4)*mEntry[1]*mEntry[1]);
    Real eigVal0 = ((Real)0.5)*(trace - discr);
    Real eigVal1 = ((Real)0.5)*(trace + discr);
    diag.MakeDiagonal(eigVal0, eigVal1);

    Real cs, sn;
    if (diff >= (Real)0)
    {
        cs = mEntry[1];
        sn = eigVal0 - mEntry[0];
    }
    else
    {
        cs = eigVal0 - mEntry[3];
        sn = mEntry[1];
    }
    Real invLength = 1.0/sqrt(cs*cs + sn*sn);
    cs *= invLength;
    sn *= invLength;

    rot.mEntry[0] = cs;
    rot.mEntry[1] = -sn;
    rot.mEntry[2] = sn;
    rot.mEntry[3] = cs;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Matrix2<Real> operator* (Real scalar, const Matrix2<Real>& mat)
{
    return mat*scalar;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Vector2 operator* (const Vector2& vec,
    const Matrix2<Real>& mat)
{
    return Vector2
    (
        vec[0]*mat[0][0] + vec[1]*mat[1][0],
        vec[0]*mat[0][1] + vec[1]*mat[1][1]
    );
}
//----------------------------------------------------------------------------


typedef Matrix2<float> Matrix2f;
typedef Matrix2<double> Matrix2d;


template<> const Matrix2<float> Matrix2<float>::ZERO(true);
template<> const Matrix2<float> Matrix2<float>::IDENTITY(false);

template<> const Matrix2<double> Matrix2<double>::ZERO(true);
template<> const Matrix2<double> Matrix2<double>::IDENTITY(false);
