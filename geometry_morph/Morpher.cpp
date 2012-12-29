#include "Morpher.h"


#define SIGN(x) (((x) < 0) ? (-1) : (((x) > 0) ? 1 : 0))
#define ROTATE_VEC(u, v, theta, axis) (u = v * cos(theta) + cross(axis, v) * sin(theta) + axis * dot(axis, v) * (1 - cos(theta)))
#define VEC_FROM_POINTS(a,b,c) \
	(a)[0] = (b)[0] - (c)[0];	\
	(a)[1] = (b)[1] - (c)[1];	\
	(a)[2] = (b)[2] - (c)[2];

Morpher::Morpher(SurfaceMeshModel *mesh1, 
	SurfaceMeshModel *mesh2, 
	NURBSCurve curve1, 
	NURBSCurve curve2, 
	QObject *parent)
	: QObject(parent)
{
	source_mesh=mesh1;
	target_mesh=mesh2;
	
	source_curve=curve1;
	target_curve=curve2;

	get_faces(source_mesh, target_mesh, source_faces, target_faces);
}

Morpher::Morpher(SurfaceMeshModel *mesh1, SurfaceMeshModel *mesh2, Structure::Graph graph1, Structure::Graph graph2)
{
	source_mesh=mesh1;
	target_mesh=mesh2;

	source_graph=graph1;
	target_graph=graph2;

	get_faces(source_mesh, target_mesh, source_faces, target_faces);
}

Morpher::~Morpher()
{

}

void Morpher::get_faces(SurfaceMeshModel *source_mesh, SurfaceMeshModel *target_mesh, std::vector<std::vector<Vector3>> &source_faces, std::vector<std::vector<Vector3>> &target_faces)
{
	Vector3VertexProperty source_points = source_mesh->vertex_property<Vector3>(VPOINT);
	Vector3VertexProperty target_points = target_mesh->vertex_property<Vector3>(VPOINT);
	// Go over faces
	foreach(Face f, source_mesh->faces())
	{
		// Collect its points
		Surface_mesh::Vertex_around_face_circulator vit = source_mesh->vertices(f),vend=vit;
		std::vector<Vector3> face_vertices;
		do{ face_vertices.push_back(source_points[vit]); } while(++vit != vend);
		source_faces.push_back(face_vertices);
	}

	// Go over faces
	foreach(Face f, target_mesh->faces())
	{
		// Collect its points
		Surface_mesh::Vertex_around_face_circulator vit = target_mesh->vertices(f),vend=vit;
		std::vector<Vector3> face_vertices;
		do{ face_vertices.push_back(target_points[vit]); } while(++vit != vend);
		target_faces.push_back(face_vertices);
	}
}

void Morpher::generateInBetween(int numStep, int timeResolution, int angleResotuion)
{
	//std::vector<std::vector<Vector3>> source_crossSections=crossSectionResampling(source_faces,source_curve,timeResolution,angleResotuion);
	//std::vector<std::vector<Vector3>> target_crossSections=crossSectionResampling(target_faces,target_curve,timeResolution,angleResotuion);

	std::vector<std::vector<Vector3>> souce_sampleDirections, target_sampleDirections;
	std::vector<std::vector<double>> souce_radii, target_radii;

	//transverse the graph nodes
	//do cross section resampling to curve nodes
	for(int i=0;i<source_graph.nodes.size();i++)
	{
		if(source_graph.nodes[i]->type()==Structure::CURVE&&target_graph.nodes[i]->type()==Structure::CURVE)
			{
				source_curve=NURBSCurve(source_graph.nodes[i]->controlPoints(),source_graph.nodes[i]->controlWeights());
				target_curve=NURBSCurve(target_graph.nodes[i]->controlPoints(),target_graph.nodes[i]->controlWeights());
			}	
	}

	std::vector<std::vector<Vector3>> source_crossSections=crossSectionResampling2(source_faces,source_curve,timeResolution,angleResotuion,souce_sampleDirections,souce_radii);
	std::vector<std::vector<Vector3>> target_crossSections=crossSectionResampling2(target_faces,target_curve,timeResolution,angleResotuion,target_sampleDirections,target_radii);

	align_sampleStartDirection(source_crossSections,target_crossSections,souce_sampleDirections,target_sampleDirections,souce_radii,target_radii);

	//debug
/*	
	int num_timeSamplings=source_crossSections.size();
	int num_angleSamplings=source_crossSections[0].size();

	for(int i=0;i<num_timeSamplings;i++)
	{
		for(int j=0;j<num_angleSamplings;j++)
		{
			target_crossSections[i][j]=2*target_crossSections[i][j];
		}
	}
*/
	saveTriMesh("resampled_source.off",source_crossSections);
	saveTriMesh("resampled_target.off", target_crossSections);

	linear_Interp_Corrd(source_crossSections,target_crossSections,numStep);
//	linear_Interp_Radius(souce_sampleDirections,target_sampleDirections,souce_radii,target_radii,numStep);



//	SurfaceMeshModel* blend_mesh=mesh_from_crossSections(source_crossSections);
}

void Morpher::align_sampleStartDirection(
	std::vector<std::vector<Vector3>> &source_crossSections, 
	std::vector<std::vector<Vector3>> &target_crossSections, 
	std::vector<std::vector<Vector3>> &souce_sampleDirections, 
	std::vector<std::vector<Vector3>> &target_sampleDirections,
	std::vector<std::vector<double>> souce_radii,
	std::vector<std::vector<double>> target_radii
	)
{
	Vector3 source_startDirection=souce_sampleDirections[0][0];
	Vector3 target_startDriection=target_sampleDirections[0][0];

	int num_timeSamplings=source_crossSections.size();
	int num_angleSamplings=source_crossSections[0].size();

	if(dot(source_startDirection,target_startDriection)<0)
	{
		for(int i=0;i<num_timeSamplings;i++)
		{
			for(int j=0;j<num_angleSamplings;j++)
			{
				target_crossSections[i][j]=target_crossSections[i][j]-2*target_radii[i][j]*target_sampleDirections[i][j];
			}
		}
	}
}

void Morpher::linear_Interp_Corrd(std::vector<std::vector<Vector3>> source_crossSections, std::vector<std::vector<Vector3>> target_crossSections, int numStep)
{
	for(int step=0; step<=numStep;step++)
	{
		std::vector<std::vector<Vector3>> blend_crossSections;
		double blend_time=step*1.0/numStep;

		for(int i=0;i<source_crossSections.size();i++)
		{
			std::vector<Vector3> cross_section;
			for(int j=0;j<source_crossSections[0].size();j++)
			{
				cross_section.push_back((1-blend_time)*source_crossSections[i][j]+blend_time*target_crossSections[i][j]);
			}
			blend_crossSections.push_back(cross_section);
		}

		QString seq_num; seq_num.sprintf("%02d", step);
		saveTriMesh(QString("blend_cylinder_%1.off").arg(seq_num).toStdString(), blend_crossSections);
	}
}

void Morpher::linear_Interp_Radius(std::vector<std::vector<Vector3>> souce_sampleDirections, 
	std::vector<std::vector<Vector3>> target_sampleDirections, 
	std::vector<std::vector<double>> souce_radii, 
	std::vector<std::vector<double>> radii,
	int numStep
	)
{

}

std::vector<std::vector<Vector3>> Morpher::crossSectionResampling(std::vector<std::vector<Vector3>> mesh_faces, NURBSCurve pathCurve,
	int timeResolution, int angleResolution)
{
	//SurfaceMeshModel *resampled_mesh=new SurfaceMeshModel();
	
	double delta_t=1.0/timeResolution;

	std::vector<Vector3> cross_section;
	std::vector<std::vector<Vector3>> crossSections;

	// set initial sampling direction
	Vector3 start_direction;
	Vector3 current_position;
	Vector3 planeNormal;
	Vector3 curveNormal;

	pathCurve.GetFrame(1*delta_t,current_position,planeNormal,curveNormal,start_direction);

	for (int i=1;i<timeResolution;i++)
	{
			cross_section=resampleCoutourPoints(mesh_faces,pathCurve,i*delta_t,angleResolution,start_direction);
			crossSections.push_back(cross_section);
	}

	return crossSections;
}

std::vector<Vector3> Morpher::resampleCoutourPoints(std::vector<std::vector<Vector3>> mesh_faces, NURBSCurve pathCurve,
	double current_time, int angleResolution, Vector3 &previous_startDirection)
{
	double delta_angle=2*M_PI/angleResolution;
	std::vector<Vector3> intersections;

	Vector3 current_position;
	Vector3 planeNormal; // tangent of curve
	Vector3 sample_startDirection; // binomal of curve
	Vector3 curveNormal;

	pathCurve.GetFrame(current_time,current_position,planeNormal,curveNormal,sample_startDirection);

	// at some time points, the frame may change its directions
	// in this situation, check the sample direction with the previous sample directions
	if(dot(previous_startDirection,sample_startDirection)<0)
	{
		sample_startDirection=-1*sample_startDirection;
		//planeNormal=-1*planeNormal;
	}

	previous_startDirection=sample_startDirection;

	Vector3 sampleDirection=sample_startDirection;
	
	for (int i=0;i<angleResolution;i++)
	{
		Vector3 intersect_point;
		double rotate_angle=i*delta_angle;
		double radius;
		
		//Vector3 rayPoint=current_position+sample_startDirection;
		//Vector3 rotated_rayPoint=RotateAround(rayPoint,current_position,planeNormal,rotate_angle);
		//sampleDirection=rotated_rayPoint-current_position;
		//sampleDirection=RotateAround(sample_startDirection,current_position,planeNormal,rotate_angle);

		sampleDirection=ROTATE_VEC(sampleDirection, sample_startDirection, rotate_angle, planeNormal);

		for(int i=0;i<mesh_faces.size();i++)
		{
			if(rayIntersectsTriangle(current_position,sampleDirection, mesh_faces[i],intersect_point,radius))
				{
					intersections.push_back(intersect_point);
					break;
				}
		}
	}

	return intersections;
}


//Modified by http://www.lighthouse3d.com/tutorials/maths/ray-triangle-intersection/ which is based on Moller and Trumbore 1997
//point(u,v) = (1-u-v)*p0 + u*p1 + v*p2	where
//	p0,p1,p2 are the vertices of the triangle 
//	u >= 0
//	v >= 0
//	u + v <= 1.0

//We also know that the parametric equation of the line is:
//	point(t) = p + t * d	where 
//	p is a point in the line
//	d is a vector that provides the line's direction

bool Morpher::rayIntersectsTriangle(Vector3 origin, Vector3 direction, std::vector<Vector3> triangle, Vector3 &intesect_point, double &radius)
{
	Vector3 e1,e2,h,s,q;
	float a,f,u,v,t;
	Vector3 v0=triangle[0];
	Vector3 v1=triangle[1];
	Vector3 v2=triangle[2];
	VEC_FROM_POINTS(e1,v1,v0);
	VEC_FROM_POINTS(e2,v2,v0);


	//crossProduct(h,direction,e2);
	h=cross(direction,e2);
	a = dot(e1,h);

	if (a > -0.00001 && a < 0.00001)
		return(false);

	f = 1/a;
	VEC_FROM_POINTS(s,origin,v0);
	u = f * (dot(s,h));

	if (u < 0.0 || u > 1.0)
		return(false);

	q=cross(s,e1);
	v = f * dot(direction,q);

	if (v < 0.0 || u + v > 1.0)
		return(false);

	// at this stage we can compute t to find out where
	// the intersection point is on the line
	t = f *dot(e2,q);

	if (t > 0.01) // ray intersection
	{
		intesect_point[0]=origin[0]+t*direction[0];
		intesect_point[1]=origin[1]+t*direction[1];
		intesect_point[2]=origin[2]+t*direction[2];

//		intesect_point[0]=origin[0]+(1-u-v)*v0[0]+u*v1[0]+v*v2[0];
//		intesect_point[1]=origin[1]+(1-u-v)*v0[1]+u*v1[1]+v*v2[1];
//		intesect_point[2]=origin[2]+(1-u-v)*v0[2]+u*v1[2]+v*v2[2];

		radius=t*dot(direction,direction);
		return(true);
	}
	else // this means that there is a line intersection
		// but not a ray intersection
		return (false);

}

SurfaceMeshModel* Morpher::mesh_from_crossSections(std::vector<std::vector<Vector3>> crossSections)
{
	SurfaceMeshModel* generatedMesh=NULL;
	std::vector<Vertex> vertices_idx;
	int timeResolution=crossSections.size();
	int angleResolution=crossSections[0].size();

	generatedMesh = new SurfaceMeshModel("test.off", "outputMeshFromCS");

	qDebug() << (QString("timeResolution=%1 angleResolution=%2 ")
		.arg(timeResolution).arg(angleResolution));

	for(int current_time=0; current_time<timeResolution;current_time++)
	{
		for (int current_angle=0;current_angle<angleResolution;current_angle++)
		{
			vertices_idx.push_back(generatedMesh->add_vertex(crossSections[current_time][current_angle]));
		}
	}


	for(int current_time=0; current_time<timeResolution-1;current_time++)
	{
		for (int current_angle=0;current_angle<angleResolution-1;current_angle++)
		{
			// this is problem with indices order
			std::vector<Vertex> face_vertex_idx;
			face_vertex_idx.push_back(vertices_idx[current_time*angleResolution+current_angle]);
			face_vertex_idx.push_back(vertices_idx[current_time*angleResolution+current_angle+1]);
			face_vertex_idx.push_back(vertices_idx[(current_time+1)*angleResolution+current_angle+1]);
			face_vertex_idx.push_back(vertices_idx[(current_time+1)*angleResolution+current_angle]);

			generatedMesh->add_face(face_vertex_idx);
		}

		std::vector<Vertex> face_vertex_idx;
		face_vertex_idx.push_back(vertices_idx[current_time*angleResolution+angleResolution-1]);
		face_vertex_idx.push_back(vertices_idx[current_time*angleResolution]);
		face_vertex_idx.push_back(vertices_idx[(current_time+1)*angleResolution]);
		face_vertex_idx.push_back(vertices_idx[(current_time+1)*angleResolution+angleResolution-1]);
		generatedMesh->add_face(face_vertex_idx);
	}

	return generatedMesh;
}

void Morpher::saveQuadMesh(std::string filename, std::vector<std::vector<Vector3>> crossSections)
{
	int num_timeSamplings=crossSections.size();
	int num_angleSamplings=crossSections[0].size();

	FILE* fp;
	fp=fopen(filename.data(),"w");

	if(fp==NULL)
	{
		exit(-1);
	}

	fprintf(fp, "OFF\n");
	fprintf(fp, "%d %d %d\n", num_timeSamplings*num_angleSamplings, (num_timeSamplings-1)*num_angleSamplings, 0 );

	for(int current_time=0; current_time<num_timeSamplings;current_time++)
	{
		for (int current_angle=0;current_angle<num_angleSamplings;current_angle++)
		{
			fprintf(fp, "%f %f %f\n", 
				crossSections[current_time][current_angle][0], 
				crossSections[current_time][current_angle][1],
				crossSections[current_time][current_angle][2]
			);

			//vertices_idx.push_back(generatedMesh->add_vertex(crossSections[current_time][current_angle]));
			//Point p;
		}
	}

	for(int current_time=0; current_time<num_timeSamplings-1;current_time++)
	{
		for (int current_angle=0;current_angle<num_angleSamplings-1;current_angle++)
		{
			std::vector<Vertex> face_vertex_idx;
			fprintf(fp, "%d %d %d %d %d\n",
				4,
				current_time*num_angleSamplings+current_angle,
				current_time*num_angleSamplings+current_angle+1,
				(current_time+1)*num_angleSamplings+current_angle+1,
				(current_time+1)*num_angleSamplings+current_angle
				);
		}

		fprintf(fp, "%d %d %d %d %d\n",
			4,
			current_time*num_angleSamplings+num_angleSamplings-1,
			current_time*num_angleSamplings,
			(current_time+1)*num_angleSamplings,
			(current_time+1)*num_angleSamplings+num_angleSamplings-1
			);

	}

	fclose(fp);
}

void Morpher::saveTriMesh(std::string filename, std::vector<std::vector<Vector3>> crossSections)
{
	int num_timeSamplings=crossSections.size();
	int num_angleSamplings=crossSections[0].size();

	FILE* fp;
	fp=fopen(filename.data(),"w");

	if(fp==NULL)
	{
		exit(-1);
	}

	fprintf(fp, "OFF\n");
	fprintf(fp, "%d %d %d\n", num_timeSamplings*num_angleSamplings, (num_timeSamplings-1)*num_angleSamplings*2, 0 );

	for(int current_time=0; current_time<num_timeSamplings;current_time++)
	{
		for (int current_angle=0;current_angle<num_angleSamplings;current_angle++)
		{
			fprintf(fp, "%f %f %f\n", 
				crossSections[current_time][current_angle][0], 
				crossSections[current_time][current_angle][1],
				crossSections[current_time][current_angle][2]
			);

			//vertices_idx.push_back(generatedMesh->add_vertex(crossSections[current_time][current_angle]));
			//Point p;
		}
	}

	for(int current_time=0; current_time<num_timeSamplings-1;current_time++)
	{
		for (int current_angle=0;current_angle<num_angleSamplings-1;current_angle++)
		{
			std::vector<Vertex> face_vertex_idx;
			fprintf(fp, "%d %d %d %d\n",
				3,
				current_time*num_angleSamplings+current_angle,
				current_time*num_angleSamplings+current_angle+1,
				(current_time+1)*num_angleSamplings+current_angle+1
				);

			fprintf(fp, "%d %d %d %d\n",
				3,
				(current_time+1)*num_angleSamplings+current_angle+1,
				(current_time+1)*num_angleSamplings+current_angle,
				current_time*num_angleSamplings+current_angle
				);
				


		}

		fprintf(fp, "%d %d %d %d\n",
			3,
			current_time*num_angleSamplings+num_angleSamplings-1,
			current_time*num_angleSamplings,
			(current_time+1)*num_angleSamplings
			);

		fprintf(fp, "%d %d %d %d\n",
			3,
			(current_time+1)*num_angleSamplings,
			(current_time+1)*num_angleSamplings+num_angleSamplings-1,
			current_time*num_angleSamplings+num_angleSamplings-1
			);
	}

	fclose(fp);
}


std::vector<std::vector<Vector3>> Morpher::crossSectionResampling2(
	std::vector<std::vector<Vector3>> mesh_faces, 
	NURBSCurve pathCurve,
	int timeResolution, 
	int angleResolution,
	std::vector<std::vector<Vector3>> &sampleDirections,
	std::vector<std::vector<double>> &radii)
{

	double delta_t=1.0/timeResolution;

	std::vector<Vector3> cross_section;
	std::vector<std::vector<Vector3>> crossSections;

	// set initial sampling direction
	Vector3 start_direction;
	Vector3 current_position;
	Vector3 planeNormal;
	Vector3 curveNormal;
	Vector3 biNormal;

	pathCurve.GetFrame(1*delta_t,current_position,planeNormal,curveNormal,biNormal);
	
	start_direction=curveNormal;

	for (int i=1;i<timeResolution;i++)
	{
		std::vector<Vector3> oneRing_sampleDirections;
		std::vector<double> oneRing_radii;

		cross_section=resampleCoutourPoints2(mesh_faces,pathCurve,i*delta_t,angleResolution,start_direction,oneRing_sampleDirections,oneRing_radii);
		crossSections.push_back(cross_section);

		sampleDirections.push_back(oneRing_sampleDirections);
		radii.push_back(oneRing_radii);
	}

	return crossSections;
}

std::vector<Vector3> Morpher::resampleCoutourPoints2(std::vector<std::vector<Vector3>> mesh_faces, NURBSCurve pathCurve,
	double current_time, int angleResolution, Vector3 &previous_startDirection,
	std::vector<Vector3> &oneRing_sampleDirections, std::vector<double> &oneRing_radii)
{
	double delta_angle=2*M_PI/angleResolution;
	double radius;
	std::vector<Vector3> intersections;

	Vector3 current_position;
	Vector3 planeNormal; // tangent of curve
	Vector3 sample_startDirection; // binomal of curve
	Vector3 biNormal;
	Vector3 curveNormal;

	pathCurve.GetFrame(current_time,current_position,planeNormal,curveNormal,biNormal);

	sample_startDirection=curveNormal;

	// at some time points, the frame may change its directions
	// in this situation, check the sample direction with the previous sample directions
	if(dot(previous_startDirection,sample_startDirection)<0)
	{
		sample_startDirection=-1*sample_startDirection;
		//planeNormal=-1*planeNormal;
	}

	previous_startDirection=sample_startDirection;

	Vector3 sampleDirection=sample_startDirection;

	for (int i=0;i<angleResolution;i++)
	{
		Vector3 intersect_point;
		double rotate_angle=i*delta_angle;

		//Vector3 rayPoint=current_position+sample_startDirection;
		//Vector3 rotated_rayPoint=RotateAround(rayPoint,current_position,planeNormal,rotate_angle);
		//sampleDirection=rotated_rayPoint-current_position;
		//sampleDirection=RotateAround(sample_startDirection,current_position,planeNormal,rotate_angle);

		sampleDirection=ROTATE_VEC(sampleDirection, sample_startDirection, rotate_angle, planeNormal);

		for(int i=0;i<mesh_faces.size();i++)
		{
			if(rayIntersectsTriangle(current_position,sampleDirection, mesh_faces[i],intersect_point,radius))
			{
				intersections.push_back(intersect_point);
				oneRing_sampleDirections.push_back(sampleDirection);
				oneRing_radii.push_back(radius);
				break;
			}
		}
	}

	return intersections;
}