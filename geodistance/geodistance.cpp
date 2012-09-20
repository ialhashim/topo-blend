#include "geodistance.h"
#include "GeoHeatHelper.h"

#include <qgl.h>
#include "../segment/CustomDrawObjects.h"
#include "StarlabDrawArea.h"

typedef CholmodSupernodalLLT< SparseMatrix<double> > CholmodSolver;

void geodistance::initParameters(RichParameterSet *pars)
{
    pars->addParam(new RichFloat("timeScale", 1.0, "Time scaling"));
    pars->addParam(new RichInt("sourceVertex", 0, "Source vertex"));
}

void geodistance::applyFilter(RichParameterSet *pars)
{
    // DEBUG:
    PointSoup * ps = new PointSoup; VectorSoup * vs = new VectorSoup;

    // Set source
    QVector<Vertex> fromVerts;
    fromVerts.push_back( Vertex( pars->getInt("sourceVertex") ) );

    // Prerequisites for triangular meshes
    GeoHeatHelper h( mesh() );

    SparseMatrix<Scalar>    A   = h.A();
    Scalar                  t   = h.t();
    SparseMatrix<Scalar>    Lc  = h.Lc();
    VectorXd                u0  = h.u0( fromVerts );

    // DEBUG:
    t *= pars->getFloat("timeScale");

    // 1) Compute heat flow for time 't'
    CholmodSolver heat_flow;

    VectorXd u = heat_flow.compute( A + (t * Lc) ).solve( u0 );
    h.fromEigenVector(u, "v:heat");

    // 2) Evaluate vector field X
    Vector3FaceProperty fgrad = h.gradientFaces();

    // 3) Comptue distance function (solve Poisson equation)
    ScalarVertexProperty vdiv = h.divergenceVertices();
    VectorXd d = h.toEigenVector(vdiv);

    CholmodSolver poisson_solver;
    VectorXd phi = poisson_solver.compute( Lc ).solve( d );

    // DEBUG:
    Vector3VertexProperty points = h.getVector3VertexProperty(VPOINT);
    Vector3FaceProperty fcenter = h.computeFaceBarycenters();

    // ==== DEBUG heat =======
    /*VectorXd uvis = u;
    double range = u.maxCoeff() - u.minCoeff();
    foreach(Vertex v, mesh()->vertices())
        ps->addPoint(points[v], qtJetColorMap((uvis[v.idx()] - u.minCoeff()) / range));*/

    // ==== DEBUG gradient =======
    //foreach(Face f, mesh()->faces())
    //    vs->addVector(fcenter[f], fgrad[f]);

    // ==== DEBUG divergence =======
    /*double minDiv = DBL_MAX, maxDiv = -DBL_MAX;
    foreach(Vertex v, mesh()->vertices()){
        minDiv = qMin(vdiv[v], minDiv);
        maxDiv = qMax(vdiv[v], maxDiv);
    }
    double div_range = maxDiv - minDiv;
    foreach(Vertex v, mesh()->vertices())
        ps->addPoint(points[v], qtJetColorMap((vdiv[v] - minDiv) / div_range) );*/

    // ==== DEBUG distance =======
    /*ScalarVertexProperty distance = h.fromEigenVector(phi, "v:distance");
    double minDist = DBL_MAX, maxDist = -DBL_MAX;
    foreach(Vertex v, mesh()->vertices()){
        minDist = qMin(distance[v], minDist);
        maxDist = qMax(distance[v], maxDist);
    }
    double distrange = maxDist - minDist;
    foreach(Vertex v, mesh()->vertices())
        ps->addPoint(points[v], qtJetColorMap((distance[v] - minDist) / distrange));*/

    drawArea()->deleteAllRenderObjects();
    drawArea()->addRenderObject(ps);
    drawArea()->addRenderObject(vs);
}

Q_EXPORT_PLUGIN(geodistance)
