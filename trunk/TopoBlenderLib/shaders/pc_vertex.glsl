#version 120

attribute vec3 vPosition;
attribute vec3 vNormal;
attribute float vSize;

varying vec4 debug_color;

uniform mat4 mvMatrix;
uniform mat4 mvpMatrix;
uniform mat3 normalMatrix;

uniform float near;
uniform float far;
uniform vec3 eyePos;
uniform float foView;
uniform int screenH;

varying vec3 normal;
varying vec3 vertex_to_light_vector;

void main()
{
	vec4 vPos = vec4(vPosition,1);
	normal = normalMatrix * vNormal;

	vec3 light_pos = vec3(0,0,0);
	vertex_to_light_vector = normalize(light_pos - vec3(mvMatrix * vPos));
		
	vec4 mod = vec4( mvMatrix * vPos );
	
	float nearTB = 2.0*tan(foView/2.0)*near;
	gl_PointSize = vSize*near*screenH/(length(eyePos-vPosition)*nearTB);

	gl_Position = mvpMatrix * vPos;
}
