#include <float.h>
#include "SurfaceMeshHelper.h"

#define M_PI       3.14159265358979323846

struct IsotropicRemesher{

	Vector3VertexProperty points;
	BoolEdgeProperty efeature;
	NanoKdTree kdtree;

	template <class T>
	static inline T deg_to_rad(const T& _angle)
	{ return M_PI*(_angle/180); }

	template <class T>
	static inline T rad_to_deg(const T& _angle)
	{ return 180*(_angle/M_PI); }

	static inline Scalar sane_aarg(Scalar _aarg){
		if (_aarg < -1)
			_aarg = -1;
		else if (_aarg >  1)
			_aarg = 1;
		return _aarg;
	}

	static inline Scalar angle(Scalar _cos_angle, Scalar _sin_angle)
	{//sanity checks - otherwise acos will return NAN
		_cos_angle = sane_aarg(_cos_angle);
		return (Scalar) _sin_angle >= 0 ? acos(_cos_angle) : -acos(_cos_angle);
	}

	// Helper function
	static Scalar calc_dihedral_angle(SurfaceMeshModel & mesh, SurfaceMeshModel::Halfedge _heh)
	{
		if (mesh.is_boundary(mesh.edge(_heh)))
		{//the dihedral angle at a boundary edge is 0
			return 0;
		}

		Vector3FaceProperty normal = mesh.face_property<Vector3>( FNORMAL );
		Vector3VertexProperty points = mesh.vertex_property<Vector3>( VPOINT );

		const Normal& n0 = normal[mesh.face(_heh)];
		const Normal& n1 = normal[mesh.face(mesh.opposite_halfedge(_heh))];

		Normal he = points[mesh.to_vertex(_heh)] - points[mesh.from_vertex(_heh)];

		Scalar da_cos = dot(n0, n1);

		//should be normalized, but we need only the sign
		Scalar da_sin_sign = dot(cross(n0, n1), he);
		return angle(da_cos, da_sin_sign);
	}

	static Scalar distPointTriangleSquared( const Vector3& _p,const Vector3& _v0,const Vector3& _v1,const Vector3& _v2,Vector3& _nearestPoint )
	{
		Vector3 v0v1 = _v1 - _v0;
		Vector3 v0v2 = _v2 - _v0;
		Vector3 n = cross(v0v1, v0v2); // not normalized !
		double d = n.sqrnorm();


		// Check if the triangle is degenerated
        if (d < FLT_MIN && d > -FLT_MIN) {
			std::cerr << "distPointTriangleSquared: Degenerated triangle !\n";
			return -1.0;
		}
		double invD = 1.0 / d;


		// these are not needed for every point, should still perform
		// better with many points against one triangle
		Vector3 v1v2 = _v2 - _v1;
		double inv_v0v2_2 = 1.0 / v0v2.sqrnorm();
		double inv_v0v1_2 = 1.0 / v0v1.sqrnorm();
		double inv_v1v2_2 = 1.0 / v1v2.sqrnorm();


		Vector3 v0p = _p - _v0;
		Vector3 t = cross(v0p, n);
		double  s01, s02, s12;
		double a = dot(t, v0v2) * -invD;
		double b = dot(t, v0v1) * invD;


		if (a < 0)
		{
			// Calculate the distance to an edge or a corner vertex
			s02 = dot( v0v2, v0p ) * inv_v0v2_2;
			if (s02 < 0.0)
			{
				s01 = dot( v0v1, v0p ) * inv_v0v1_2;
				if (s01 <= 0.0) {
					v0p = _v0;
				} else if (s01 >= 1.0) {
					v0p = _v1;
				} else {
					v0p = _v0 + v0v1 * s01;
				}
			} else if (s02 > 1.0) {
				s12 = dot( v1v2, ( _p - _v1 )) * inv_v1v2_2;
				if (s12 >= 1.0) {
					v0p = _v2;
				} else if (s12 <= 0.0) {
					v0p = _v1;
				} else {
					v0p = _v1 + v1v2 * s12;
				}
			} else {
				v0p = _v0 + v0v2 * s02;
			}
		} else if (b < 0.0) {
			// Calculate the distance to an edge or a corner vertex
			s01 = dot( v0v1, v0p ) * inv_v0v1_2;
			if (s01 < 0.0)
			{
				s02 = dot( v0v2,  v0p ) * inv_v0v2_2;
				if (s02 <= 0.0) {
					v0p = _v0;
				} else if (s02 >= 1.0) {
					v0p = _v2;
				} else {
					v0p = _v0 + v0v2 * s02;
				}
			} else if (s01 > 1.0) {
				s12 = dot( v1v2, ( _p - _v1 )) * inv_v1v2_2;
				if (s12 >= 1.0) {
					v0p = _v2;
				} else if (s12 <= 0.0) {
					v0p = _v1;
				} else {
					v0p = _v1 + v1v2 * s12;
				}
			} else {
				v0p = _v0 + v0v1 * s01;
			}
		} else if (a+b > 1.0) {
			// Calculate the distance to an edge or a corner vertex
			s12 = dot( v1v2, ( _p - _v1 )) * inv_v1v2_2;
			if (s12 >= 1.0) {
				s02 = dot( v0v2, v0p ) * inv_v0v2_2;
				if (s02 <= 0.0) {
					v0p = _v0;
				} else if (s02 >= 1.0) {
					v0p = _v2;
				} else {
					v0p = _v0 + v0v2*s02;
				}
			} else if (s12 <= 0.0) {
				s01 = dot( v0v1,  v0p ) * inv_v0v1_2;
				if (s01 <= 0.0) {
					v0p = _v0;
				} else if (s01 >= 1.0) {
					v0p = _v1;
				} else {
					v0p = _v0 + v0v1 * s01;
				}
			} else {
				v0p = _v1 + v1v2 * s12;
			}
		} else {
			// Calculate the distance to an interior point of the triangle
			_nearestPoint = _p - n*(dot(n,v0p) * invD);
			return (_nearestPoint - _p).sqrnorm();
		}

		_nearestPoint = v0p;

		return (_nearestPoint - _p).sqrnorm();
	}

	void remesh(double targetEdgeLength, int numIterations )
	{
		const double low  = (4.0 / 5.0) * targetEdgeLength;
		const double high = (4.0 / 3.0) * targetEdgeLength;

		// Copy original mesh
		SurfaceMeshModel* copy = new SurfaceMeshModel(mesh->path,"original");
		copy->read( mesh->path.toStdString() );

		// Build KD-tree
		kdtree.cloud.pts.clear();
		foreach(Vertex v, mesh->vertices())
			kdtree.addPoint(points[v]);
		kdtree.build();

		for(int i = 0; i < numIterations; i++)
		{
			splitLongEdges(high);
			collapseShortEdges(low, high);
			equalizeValences();
			tangentialRelaxation();
			//projectToSurface(copy);
		}
	}

	/// performs edge splits until all edges are shorter than the threshold
	void splitLongEdges(double maxEdgeLength )
	{
		const double maxEdgeLengthSqr = maxEdgeLength * maxEdgeLength;

		SurfaceMeshModel::Edge_iterator e_it;
		SurfaceMeshModel::Edge_iterator e_end = mesh->edges_end();

		// iterate over all edges
		for (e_it = mesh->edges_begin(); e_it != e_end; ++e_it){
			const SurfaceMeshModel::Halfedge & hh = mesh->halfedge( e_it, 0 );

			const SurfaceMeshModel::Vertex & v0 = mesh->from_vertex(hh);
			const SurfaceMeshModel::Vertex & v1 = mesh->to_vertex(hh);

			Vector3 vec = points[v1] - points[v0];

			// edge to long?
			if ( vec.sqrnorm() > maxEdgeLengthSqr ){

				const Vector3 midPoint = points[v0] + ( 0.5 * vec );

				// split at midpoint
				SurfaceMeshModel::Vertex vh = mesh->add_vertex( midPoint );

				bool hadFeature = efeature[e_it];

				mesh->split(e_it, vh);

				if ( hadFeature )
				{
					foreach(Halfedge e, mesh->onering_hedges(vh))
					{
						if ( mesh->to_vertex(e) == v0 || mesh->to_vertex(e) == v1 )
						{
							efeature[mesh->edge(e)] = true;
						}
					}
				}
			}
		}
	}

	/// collapse edges shorter than minEdgeLength if collapsing doesn't result in new edge longer than maxEdgeLength
	void collapseShortEdges(const double _minEdgeLength, const double _maxEdgeLength )
	{
		const double _minEdgeLengthSqr = _minEdgeLength * _minEdgeLength;
		const double _maxEdgeLengthSqr = _maxEdgeLength * _maxEdgeLength;

		//add checked property
		BoolEdgeProperty checked = mesh->edge_property< bool >("e:checked", false);

		SurfaceMeshModel::Edge_iterator e_it;
		SurfaceMeshModel::Edge_iterator e_end = mesh->edges_end();

		bool finished = false;

		while( !finished ){

			finished = true;

			for (e_it = mesh->edges_begin(); e_it != mesh->edges_end() ; ++e_it){

				if ( checked[e_it] )
					continue;

				checked[e_it] = true;

				const SurfaceMeshModel::Halfedge & hh = mesh->halfedge( e_it, 0 );

				const SurfaceMeshModel::Vertex & v0 = mesh->from_vertex(hh);
				const SurfaceMeshModel::Vertex & v1 = mesh->to_vertex(hh);

				const Vector3 vec = points[v1] - points[v0];

				const double edgeLength = vec.sqrnorm();

				// edge too short but don't try to collapse edges that have length 0
				if ( (edgeLength < _minEdgeLengthSqr) && (edgeLength > DBL_EPSILON) ){

					//check if the collapse is ok
					const Vector3 & B = points[v1];

					bool collapse_ok = true;

					foreach( Halfedge hvit, mesh->onering_hedges(v0) )
					{
						double d = (B - points[ mesh->to_vertex(hvit) ]).sqrnorm();

						if ( d > _maxEdgeLengthSqr || mesh->is_boundary( mesh->edge( hvit ) ) || efeature[mesh->edge(hvit)] )
						{
							collapse_ok = false;
							break;
						}
					}

					if( collapse_ok && mesh->is_collapse_ok(hh) ) {
						mesh->collapse( hh );

						finished = false;
					}
				}
			}
		}

		mesh->remove_edge_property(checked);

		mesh->garbage_collection();
	}

	void equalizeValences(  )
	{
		SurfaceMeshModel::Edge_iterator e_it;
		SurfaceMeshModel::Edge_iterator e_end = mesh->edges_end();

		for (e_it = mesh->edges_begin(); e_it != e_end; ++e_it){

			if ( !mesh->is_flip_ok(e_it) ) continue;
			if ( efeature[e_it] ) continue;

			const SurfaceMeshModel::Halfedge & h0 = mesh->halfedge( e_it, 0 );
			const SurfaceMeshModel::Halfedge & h1 = mesh->halfedge( e_it, 1 );

			if (h0.is_valid() && h1.is_valid())
			{
				if (mesh->face(h0).is_valid() && mesh->face(h1).is_valid()){
					//get vertices of corresponding faces
					const SurfaceMeshModel::Vertex & a = mesh->to_vertex(h0);
					const SurfaceMeshModel::Vertex & b = mesh->to_vertex(h1);
					const SurfaceMeshModel::Vertex & c = mesh->to_vertex(mesh->next_halfedge(h0));
					const SurfaceMeshModel::Vertex & d = mesh->to_vertex(mesh->next_halfedge(h1));

					const int deviation_pre =  abs((int)(mesh->valence(a) - targetValence(a)))
							+abs((int)(mesh->valence(b) - targetValence(b)))
							+abs((int)(mesh->valence(c) - targetValence(c)))
							+abs((int)(mesh->valence(d) - targetValence(d)));
					mesh->flip(e_it);

					const int deviation_post = abs((int)(mesh->valence(a) - targetValence(a)))
							+abs((int)(mesh->valence(b) - targetValence(b)))
							+abs((int)(mesh->valence(c) - targetValence(c)))
							+abs((int)(mesh->valence(d) - targetValence(d)));

					if (deviation_pre <= deviation_post)
						mesh->flip(e_it);
				}
			}
		}
	}

	///returns 4 for boundary vertices and 6 otherwise
	inline int targetValence(const SurfaceMeshModel::Vertex& _vh ){
		if (isBoundary(_vh))
			return 4;
		else
			return 6;
	}

	inline bool isBoundary(const SurfaceMeshModel::Vertex& _vh )
	{
		foreach( Halfedge hvit, mesh->onering_hedges(_vh) )
		{
			if ( mesh->is_boundary( mesh->edge( hvit ) ) )
				return true;
		}
		return false;
	}

	inline bool isFeature(const SurfaceMeshModel::Vertex& _vh )
	{
		foreach( Halfedge hvit, mesh->onering_hedges(_vh) )
		{
			if(efeature[mesh->edge(hvit)])
				return true;
		}

		return false;
	}

	void tangentialRelaxation(  )
	{
		mesh->update_face_normals();
		mesh->update_vertex_normals();

		Vector3VertexProperty q = mesh->vertex_property<Vector3>("v:q");
		Vector3VertexProperty normal = mesh->vertex_property<Vector3>(VNORMAL);

		SurfaceMeshModel::Vertex_iterator v_it;
		SurfaceMeshModel::Vertex_iterator v_end = mesh->vertices_end();

		//first compute barycenters
		for (v_it = mesh->vertices_begin(); v_it != v_end; ++v_it){

			Vector3 tmp(0);
			uint N = 0;

			foreach( Halfedge hvit, mesh->onering_hedges(v_it) )
			{
				tmp += points[ mesh->to_vertex(hvit) ];
				N++;
			}

			if (N > 0)
				tmp /= (double) N;

			q[v_it] = tmp;
		}

		//move to new position
		for (v_it = mesh->vertices_begin(); v_it != v_end; ++v_it)
		{
			if ( !isBoundary(v_it) && !isFeature(v_it) )
			{
				//Vector3 newPos = q[v_it] + (dot(normal[v_it], (points[v_it] - q[v_it]) ) * normal[v_it]);
				points[v_it] = q[v_it];
			}
		}

		mesh->remove_vertex_property(q);
	}

	Vector3 findNearestPoint(SurfaceMeshModel * orginal_mesh, const Vector3& _point, SurfaceMeshModel::Face& _fh, double* _dbest)
	{
		Vector3VertexProperty orig_points = orginal_mesh->vertex_property<Vector3>( VPOINT );

		Vector3  p_best = orig_points[ Vertex(0) ];
		SurfaceMeshModel::Scalar d_best = (_point - p_best).sqrnorm();

		SurfaceMeshModel::Face fh_best;

		// exhaustive search
		SurfaceMeshModel::Face_iterator cf_it  = orginal_mesh->faces_begin();
		SurfaceMeshModel::Face_iterator cf_end = orginal_mesh->faces_end();

		KDResults matches;
		kdtree.k_closest(_point, 32, matches);

		foreach(KDResultPair match, matches)
		{
			int vi = match.first;

			foreach(Halfedge h, orginal_mesh->onering_hedges(Vertex(vi)))
			{
				SurfaceMeshModel::Vertex_around_face_circulator cfv_it = orginal_mesh->vertices( orginal_mesh->face(h) );

				// Assume triangular
				const Vector3& pt0 = orig_points[   cfv_it];
				const Vector3& pt1 = orig_points[ ++cfv_it];
				const Vector3& pt2 = orig_points[ ++cfv_it];

				Vector3 ptn;

				SurfaceMeshModel::Scalar d = distPointTriangleSquared( _point, pt0, pt1, pt2, ptn );

				if( d < d_best)
				{
					d_best = d;
					p_best = ptn;

					fh_best = cf_it;
				}
			}
		}

		// return face
		_fh = fh_best;

		// return distance
		if(_dbest)
			*_dbest = sqrt(d_best);

		return p_best;
	}

	void projectToSurface(SurfaceMeshModel * orginal_mesh )
	{
		SurfaceMeshModel::Vertex_iterator v_it;
		SurfaceMeshModel::Vertex_iterator v_end = mesh->vertices_end();

		for (v_it = mesh->vertices_begin(); v_it != v_end; ++v_it)
		{
			if (isBoundary(v_it)) continue;
			if ( isFeature(v_it)) continue;

			Vector3 p = points[v_it];
			SurfaceMeshModel::Face fhNear;
			double distance;

			Vector3 pNear = findNearestPoint(orginal_mesh, p, fhNear, &distance);

			points[v_it] = pNear;
		}
	}

	void doRemesh( Scalar longest_edge_length, int num_split_iters = 10, double angleDeg = 44.0 )
	{
		points = mesh->vertex_property<Vector3>( VPOINT );

		// Prepare for sharp features
		efeature = mesh->edge_property<bool>("e:feature", false);
		if( angleDeg )
		{
			double angleThreshold = deg_to_rad(angleDeg);

			foreach(Edge e, mesh->edges())
			{
				if (abs(calc_dihedral_angle(*mesh, mesh->halfedge(e,0))) > angleThreshold)
					efeature[e] = true;
			}
		}

		/// Perform refinement
		this->remesh(longest_edge_length, num_split_iters);

		// Clean up
		mesh->remove_edge_property(efeature);
	}
	
	IsotropicRemesher(SurfaceMeshModel * useMesh){
		this->mesh = useMesh;
	}
	
	SurfaceMeshModel * mesh;
};
