varying vec3 N;
varying vec3 v;

void main() {
	//get the vertices into eye space
	v = (gl_ModelViewMatrix*gl_Vertex).xyz;

	//get the normals into eye space
	N = normalize(gl_Normal);
	N = gl_NormalMatrix*N;
	
	gl_Position = ftransform();
}