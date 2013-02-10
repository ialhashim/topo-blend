varying vec3 normal;
varying vec3 vertex_to_light_vector;
varying vec4 debug_color;

void main()
{
	const vec4 ambientColor = vec4(0.05, 0.05, 0.05, 1.0);
	const vec4 diffuseColor = vec4(1.0, 1.0, 1.0, 1.0);

	vec3 normalized_normal = normalize(normal); // renormalization

	float diffuseTerm = clamp(dot(normal, vertex_to_light_vector), 0.0, 1.0);

	gl_FragColor = ambientColor + diffuseColor * diffuseTerm;

	//gl_FragColor = debug_color;

	if(dot(gl_PointCoord-0.5,gl_PointCoord-0.5)>0.25) 
		discard;
}
