varying vec3 e;
varying vec3 n;

void main() {
  	vec4 pos = vec4(gl_Vertex);
	e = normalize(vec4(gl_ModelViewMatrix*pos).xyz);
    n = gl_NormalMatrix*normalize(gl_Normal.xyz); 
	gl_Position = gl_ModelViewProjectionMatrix * pos; 
}