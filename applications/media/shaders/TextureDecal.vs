#version 120

varying vec2 texCoord;

void main()
{	
	//gl_TexCoord[0] = gl_MultiTexCoord0;

	texCoord = gl_MultiTexCoord0.xy;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
} 


