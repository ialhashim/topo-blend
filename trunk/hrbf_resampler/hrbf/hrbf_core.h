/* This Source Code Form is subject to the terms of the Mozilla Public License, 
 * v. 2.0. If a copy of the MPL was not distributed with this file, 
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 */
/// @file hrbf_core.hpp
/// Original author: Gael Guennebaud - gael.guennebaud@inria.fr - http://www.labri.fr/perso/guenneba/
/// Rodolphe Vaillant - (Fixed the gradient evaluation) - http://www.irit.fr/~Rodolphe.Vaillant

#ifndef HRBF_CORE_HPP__
#define HRBF_CORE_HPP__

#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <vector>
#include <iostream>

/// @brief fitting surface on a cloud point and evaluating the implicit surface
/// @tparam _Scalar : a base type Scalar, double etc.
/// @tparam _Dim    : integer of the dimension of the ambient space 
/// (for a implicit surface == 3)
/// @tparam Rbf     : the class of a the radial basis
/// must implement Scalar Rbf::f(Scalar) Scalar Rbf::df(Scalar) Scalar Rbf::ddf(Scalar)
/// (see hrbf_phi_funcs.h for an example)
/// @note Please go see http://eigen.tuxfamily.org to use matrix and vectors 
/// types of this lib. The documentation is pretty good.
template<typename _Scalar, int _Dim, typename Rbf>
class HRBF_fit
{
public:
    typedef _Scalar Scalar;
    enum { Dim = _Dim };

    typedef Eigen::Matrix<Scalar,Dim,Dim>                       MatrixDD;
    typedef Eigen::Matrix<Scalar,Dim,1>                         Vector;
    typedef Eigen::Matrix<Scalar,Dim,Eigen::Dynamic>            MatrixDX;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> MatrixXX;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1>              VectorX;

    HRBF_fit() {}

    // --------------------------------------------------------------------------

    /// Compute surface interpolation given a set of points and normals.
    /// This solve the linear system of equation to find alpha scalars and beta
    /// beta vectors stored in '_alphas' and '_betas' attributes.
    void hermite_fit(const std::vector<Vector>& points,
                     const std::vector<Vector>& normals)
    {
        assert( points.size() == normals.size() );

        int nb_points           = points.size();
        int nb_hrbf_constraints = (Dim+1)*nb_points;
        int nb_constraints      = nb_hrbf_constraints;
        int nb_coeffs           = (Dim+1)*nb_points;

        _node_centers.resize(Dim, nb_points);
        _betas.       resize(Dim, nb_points);
        _alphas.      resize(nb_points);

        // Assemble the "design" and "value" matrix and vector
        MatrixXX  D(nb_constraints, nb_coeffs);
        VectorX   f(nb_constraints);
        VectorX   x(nb_coeffs);

        // copy the node centers
        for(int i = 0; i < nb_points; ++i)
            _node_centers.col(i) = points[i];

        for(int i = 0; i < nb_points; ++i)
        {
            Vector p = points[i];
            Vector n = normals[i];

            int io = (Dim+1) * i;
            f(io) = 0;
            f.template segment<Dim>(io + 1) = n;

            for(int j = 0; j < nb_points; ++j)
            {
                int jo = (Dim + 1) * j;
                Vector diff = p - _node_centers.col(j);
                Scalar l = diff.norm();
                if( l == 0 ) {
                    D.template block<Dim+1,Dim+1>(io,jo).setZero();
                } else {
                    Scalar w    = Rbf::f(l);
                    Scalar dw_l = Rbf::df(l)/l;
                    Scalar ddw  = Rbf::ddf(l);
                    Vector g    = diff * dw_l;
                    D(io,jo) = w;
                    D.row(io).template segment<Dim>(jo+1) = g.transpose();
                    D.col(jo).template segment<Dim>(io+1) = g;
                    D.template block<Dim,Dim>(io+1,jo+1)  = (ddw - dw_l)/(l*l) * (diff * diff.transpose());
                    D.template block<Dim,Dim>(io+1,jo+1).diagonal().array() += dw_l;
                }
            }
        }

        x = D.lu().solve(f);
        Eigen::Map< Eigen::Matrix<Scalar,Dim+1,Eigen::Dynamic> > mx( x.data(), Dim + 1, nb_points);

        _alphas = mx.row(0);
        _betas  = mx.template bottomRows<Dim>();
    }

    // -------------------------------------------------------------------------

    /// Evaluate potential f() at position 'x'
    Scalar eval(const Vector& x) const
    {
        Scalar ret = 0;
        int nb_nodes = _node_centers.cols();

        for(int i = 0; i < nb_nodes; ++i)
        {
            Vector diff = x - _node_centers.col(i);
            Scalar l    = diff.norm();

            if( l > 0 )
            {
                ret += _alphas(i) * Rbf::f(l);
                ret += _betas.col(i).dot( diff ) * Rbf::df(l) / l;
            }
        }
        return ret;
    }

    // -------------------------------------------------------------------------
#define GRAD_THRESHOLD 0.00001

    /// Evaluate gradient nabla f() at position 'x'
    Vector grad(const Vector& x) const
    {
        Vector grad = Vector::Zero();
        int nb_nodes = _node_centers.cols();
        for(int i = 0; i < nb_nodes; i++)
        {
            Vector node  = _node_centers.col(i);
            Vector beta  = _betas.col(i);
            Scalar  alpha = _alphas(i);
            Vector diff  = x - node;

            Vector diffNormalized = diff;
            Scalar l =  diff.norm();

            if( l > GRAD_THRESHOLD )
            {
                diffNormalized.normalize();
                Scalar dphi  = Rbf::df(l);
                Scalar ddphi = Rbf::ddf(l);

                Scalar alpha_dphi = alpha * dphi;

                Scalar bDotd_l = beta.dot(diff)/l;
                Scalar squared_l = diff.squaredNorm();

                grad += alpha_dphi * diffNormalized;
                grad += bDotd_l * (ddphi * diffNormalized - diff * dphi / squared_l) + beta * dphi / l ;
            }
        }
        return grad;
    }

    // --------------------------------------------------------------------------

    /// Each column represents p_i:  VectorX pi = _node_centers.col(i);
    MatrixDX  _node_centers;
    /// Vector of scalar values alpha
    VectorX   _alphas;
    /// Each column represents beta_i:  VectorX bi = _betas.col(i);
    MatrixDX  _betas;

}; // END HermiteRbfReconstruction Class =======================================

#endif // HRBF_CORE_HPP__

