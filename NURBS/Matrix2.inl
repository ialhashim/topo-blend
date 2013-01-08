
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
