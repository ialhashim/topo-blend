#pragma once
#include "SurfaceMeshHelper.h"

// Eigne matrix library
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/CholmodSupport>
using namespace Eigen;

// Missing Math
# define M_PI		3.14159265358979323846
# define M_PI_2		1.57079632679489661923
#define qRanged(min, v, max) ( qMax(min, qMin(v, max)) )

class  GeoHeatHelper : public SurfaceMeshHelper{
public:
    GeoHeatHelper(SurfaceMeshModel* mesh) : SurfaceMeshHelper(mesh)
    {
        mesh->remove_vertex_property(mesh->get_vertex_property<Scalar>("v:function"));
        mesh->remove_vertex_property(mesh->get_vertex_property<Scalar>("v:cotan"));
        mesh->remove_vertex_property(mesh->get_vertex_property<Scalar>("v:heat"));
        mesh->remove_vertex_property(mesh->get_vertex_property<Scalar>("v:divergence"));
        mesh->remove_edge_property(mesh->get_edge_property<Scalar>("e:cotan"));
        mesh->remove_face_property(mesh->get_face_property<Vector3>("f:gradient"));

        points  = this->getVector3VertexProperty(VPOINT);
        fnormal = this->computeFaceNormals();
        farea   = this->computeFaceAreas();
        varea   = this->computeVertexVoronoiArea();
    }

    Scalar t()
    {
        // t = (avg edge length) ^ 2
        double avg = 0.0;
        ScalarEdgeProperty elen = computeEdgeLengths();
        foreach(Edge e, mesh->edges()) avg += elen[e];
        avg /= mesh->n_edges();
        return avg * avg;
    }

    VectorXd u0(const QVector<Vertex> & vidx, const string property="v:function", Scalar value = 1.0)
    {
        // u0 on source vertices = 1.0 and zero elsewhere
        ScalarVertexProperty vfunction = mesh->add_vertex_property<Scalar>(property, 0);
        VectorXd u_0 = VectorXd::Zero(mesh->n_vertices());
        foreach(Vertex v, vidx)
            u_0[v.idx()] = vfunction[v] = value;
        return u_0;
    }

    SparseMatrix<Scalar> A()
    {
        // A is diagonal matrix of vertex areas
        typedef Eigen::Triplet<double> T;
        std::vector< T > A_elements;

        foreach(Vertex v, mesh->vertices())
            A_elements.push_back( T(v.idx(), v.idx(), varea[v]) );

        SparseMatrix<Scalar> matA(mesh->n_vertices(), mesh->n_vertices());
        matA.setFromTriplets(A_elements.begin(), A_elements.end());
        return matA;
    }

    SparseMatrix<Scalar> Lc(const string vproperty="v:cotan", const string eproperty="e:cotan")
    {
        ScalarEdgeProperty ecot = mesh->add_edge_property<Scalar>(eproperty, 0.0);
        ScalarVertexProperty vcot = mesh->add_vertex_property<Scalar>(vproperty, 0.0);
        Vector3VertexProperty p = getVector3VertexProperty(VPOINT);

        // Efficent sparse matrix construction
        typedef Eigen::Triplet<double> T;
        std::vector< T > L_c;

        // Fill as cotan operator
        foreach(Edge e, mesh->edges()){
            Scalar cot_alpha = 0, cot_beta = 0;

            Vertex vi = mesh->vertex(e, 0);
            Vertex vj = mesh->vertex(e, 1);

            Vertex v_a = mesh->to_vertex(mesh->next_halfedge(mesh->halfedge(e, 0)));
            if(has_halfedge(v_a, vj)) cot_alpha = dot(p[vi]-p[v_a], p[vj]-p[v_a]) / cross(p[vi]-p[v_a], p[vj]-p[v_a]).norm();

            Vertex v_b = mesh->to_vertex(mesh->next_halfedge(mesh->halfedge(e, 1)));
            if(has_halfedge(v_b, vi)) cot_beta = dot(p[vi]-p[v_b], p[vj]-p[v_b]) / cross(p[vi]-p[v_b], p[vj]-p[v_b]).norm();

            Scalar cots = 0.5 * (cot_alpha + cot_beta);

            if(abs(cots) == 0) continue;

            L_c.push_back(T(vi.idx(), vj.idx(), -cots));
            L_c.push_back(T(vj.idx(), vi.idx(), -cots));
            L_c.push_back(T(vi.idx(), vi.idx(), cots));
            L_c.push_back(T(vj.idx(), vj.idx(), cots));

            // Just for record
            vcot[vi] += cots;
            vcot[vj] += cots;
            ecot[e] = cots;
        }

        // Initialize a sparse matrix
        SparseMatrix<Scalar> Lc_mat(mesh->n_vertices(), mesh->n_vertices());
        Lc_mat.setFromTriplets(L_c.begin(), L_c.end());
        return Lc_mat;
    }

    Vector3FaceProperty gradientFaces(bool isNormalizeNegateGradient = true, const string property="f:gradient")
    {
        Vector3FaceProperty fgradient = mesh->add_face_property<Vector3>(property);
        ScalarVertexProperty u = mesh->get_vertex_property<Scalar>("v:heat");

        foreach(Face f, mesh->faces()){
            Vector3 vsum(0);

            Surface_mesh::Halfedge_around_face_circulator h(mesh, f), hend = h;
            do{
                Vector3 ei = points[mesh->from_vertex(h)] - points[mesh->from_vertex(mesh->prev_halfedge(h))];
                Vertex i = mesh->to_vertex(h);
                vsum += u[i] * cross(fnormal[f], ei);
            }while (++h != hend);

            fgradient[f] = ( 1.0 / (2.0 * farea[f]) ) * vsum;

            if(isNormalizeNegateGradient) fgradient[f] = (-fgradient[f]).normalized();
        }

        return fgradient;
    }

    ScalarVertexProperty divergenceVertices(const string property="v:divergence")
    {
        ScalarVertexProperty div = mesh->add_vertex_property<Scalar>(property);
        Vector3FaceProperty fgradient = mesh->get_face_property<Vector3>("f:gradient");

        foreach(Vertex i, mesh->vertices()){
            double sum_j = 0.0;

            Surface_mesh::Halfedge_around_vertex_circulator j(mesh, i), hend = j;
            do {
                if(!mesh->is_valid(mesh->face(j))) continue; // todo: handle this?

                // Face gradient
                Vector3 Xj = fgradient[ mesh->face(j) ];

                // Vertex position
                Vector3 pi = points[mesh->from_vertex(j)];
                Vector3 p1 = points[mesh->to_vertex(j)];
                Vector3 p2 = points[mesh->to_vertex(mesh->next_halfedge(j))];

                // Incident edges
                Vector3 e1 = p1 - pi;
                Vector3 e2 = p2 - pi;

                // Angles
                double theta1 = acos( dot((p1-p2).normalized(), (pi-p2).normalized()) );
                double theta2 = acos( dot((p2-p1).normalized(), (pi-p1).normalized()) );
                double cot1 = 1.0 / tan(theta1);
                double cot2 = 1.0 / tan(theta2);

                sum_j += (cot1 * dot(e1, Xj)) + (cot2 * dot(e2, Xj));

            } while(++j != hend);

            div[i] = 0.5 * sum_j;
        }

        return div;
    }

    VectorXd toEigenVector(const ScalarVertexProperty & vproperty){
        VectorXd V( mesh->n_vertices() );
        foreach(Vertex i, mesh->vertices()) V(i.idx()) = vproperty[i];
        return V;
    }

    ScalarVertexProperty fromEigenVector(const VectorXd & V, const string property){
        ScalarVertexProperty vprop = mesh->vertex_property<Scalar>(property);
        foreach(Vertex v, mesh->vertices())
            vprop[v] = V[v.idx()];
        return vprop;
    }

    bool has_halfedge(Vertex start, Vertex end){
        Halfedge h = mesh->halfedge(start);
        const Halfedge hh = h;
        if (h.is_valid()){
            do{
                if (mesh->to_vertex(h) == end) return true;
                h = mesh->cw_rotated_halfedge(h);
            } while (h != hh);
        }
        return false;
    }
};
