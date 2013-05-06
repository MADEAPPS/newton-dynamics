varying vec4 diffuse;
varying vec4 ambient;
varying vec3 normal;
varying vec3 lightDir;
varying vec3 halfVector;
varying vec4 textureColorOnOff;

attribute vec4 textureEnableOnOff;

void main()
{	
	// first transform the normal into eye space and normalize the result 
	normal = normalize(gl_NormalMatrix * gl_Normal);
	
	// now normalize the light's direction. Note that according to the
	// OpenGL specification, the light is stored in eye space. Also since 
	// we're talking about a directional light, the position field is actually 
	// direction 
	lightDir = normalize(vec3(gl_LightSource[0].position));

	// Normalize the halfVector to pass it to the fragment shader 
	halfVector = normalize(gl_LightSource[0].halfVector.xyz);
	
	// Compute the diffuse, ambient and globalAmbient terms 
	diffuse  = gl_FrontMaterial.diffuse * gl_LightSource[0].diffuse;
	ambient  = gl_FrontMaterial.ambient * gl_LightSource[0].ambient;
	ambient += gl_LightModel.ambient * gl_FrontMaterial.ambient;
		
	textureColorOnOff = textureEnableOnOff; 
	gl_TexCoord[0] = gl_MultiTexCoord0;
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
} 


