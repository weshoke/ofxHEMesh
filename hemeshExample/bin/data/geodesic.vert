uniform float maxDistance;
attribute float distance;

varying vec3 position;
varying vec3 normal;
varying float normalizedDistance;


void main()
{	
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	gl_FrontColor = gl_Color;
	
	position = (gl_ModelViewMatrix*gl_Vertex).xyz;
	normal = normalize(gl_Normal);
	normal = gl_NormalMatrix*normal;
	
	normalizedDistance = distance/maxDistance;
}