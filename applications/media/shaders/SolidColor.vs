

void main()
{	
	gl_FrontColor = gl_Color;
//	gl_Position = ftransform();	
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	
} 


