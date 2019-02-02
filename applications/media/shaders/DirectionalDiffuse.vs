#version 120
varying vec3 normal;
varying vec3 position;

void main()
{	
	// get normal in camera space
	normal = gl_NormalMatrix * gl_Normal;

	// get position is camera space
	position = vec3 (gl_ModelViewMatrix * gl_Vertex);

	// get position is perective space
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
} 


