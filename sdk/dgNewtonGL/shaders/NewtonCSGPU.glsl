static char* shader = "															\
#version 460 core																\
#extension GL_ARB_compute_shader : enable										\
#extension GL_ARB_shader_storage_buffer_object : enable							\
																				\
// Target 0 : Vertex Position													\
layout(std430, binding = 0) buffer Pos {										\
  vec4 Positions[ ];															\
};																				\
																				\
// Target 1 : Vertex Velocity													\
layout(std430, binding = 1) buffer Vel {										\
  vec4 Velocities[ ];															\
};																				\
																				\
// Target 2 : Vertex Body														\
layout(std430, binding = 2) buffer Bod {										\
  vec4 BodyDataA[ ];															\
  // 																			\
  /* the buffer model is like this...											\
    float life[0];																\
    float mass[1];																\
    float bounce[2];															\
    float size[3];																\
	*/  																		\
};																				\
																				\
layout (local_size_x = 1, local_size_y = 32, local_size_z = 1) in;				\
																				\
// Gravity																		\
const vec3 gravity = vec3(0, -9.81f, 0);										\
																				\
// Frame delta for calculations													\
uniform float timestep;															\
uniform vec4 objpos;															\
																				\
// Viewport dimensions for border clamp											\
uniform vec3 dgWorldSize;														\
uniform int useLimits;															\
																				\
void main() {																	\
  ivec3 ipos = ivec3(gl_GlobalInvocationID);									\
																				\
  // can use x,y,z space world, in this exemple only space y is used.			\
  uint index = ipos.y; 															\
																				\
   // Read position and velocity												\
   vec3 vPos = Positions[index].xyz;											\
   vec3 vVel = Velocities[index].xyz;											\
   vec4 vBodA = BodyDataA[index];    											\
																				\
   // Calculate new velocity depending on attraction point 						\
																				\
   vec3 vPos2 = objpos.xyz;														\
   float diss = (vPos2.x * vPos2.x + vPos2.y * vPos2.y + vPos2.z * vPos2.z); 	\
   if (diss > 0.0f){															\
    if (diss > 1.0f)															\
      vVel = vVel + (normalize(vPos2 - vPos) * 5.0f) * (timestep);   			\
   } else {  																	\
     vVel = vVel + (gravity * vBodA.y) * (timestep);	 						\
   }																			\
   																				\
   																				\
   																				\
   // Move by velocity															\
   vPos = vPos + vVel * (timestep);												\
   																				\
   if ((vPos.y > 5.0f) && (vBodA.z < 1.0f)) 									\
	 vBodA.z = 200.0f; //25.0f*vPos.y;	   										\
																				\
    if (useLimits == 1) {														\
																				\
        if (vPos.x < -dgWorldSize.x) {											\
            vPos.x = -dgWorldSize.x;											\
            vVel.x = -vVel.x;													\
        }																		\
																				\
        if (vPos.x > dgWorldSize.x) {											\
            vPos.x = dgWorldSize.x;												\
            vVel.x = -vVel.x;													\
        }																		\
		//  																	\
		if (vPos.y < 0.25f) {													\
		  if (vBodA.z > 0.0f) {													\
		    vVel.y = vVel.y + (vBodA.z)*0.01f;									\
		  	vBodA.z -= 2.0f;													\
		  } else {																\
		    vBodA.z = 0.0f;														\
		    vPos.y = 0.25f;														\
		    vVel.y = 0.0f;														\
			if (vVel.x != 0.0f){												\
			  if (vVel.x > 0.0f) vVel.x = vVel.x - 0.01f; 						\
			  if (vVel.x < 0.0f) vVel.x = vVel.x + 0.01f;						\
			  //																\
			  if (abs(vVel.x) < 0.02) vVel.x = 0.0f;							\
			} 																	\
			if (vVel.z != 0.0f){  												\
			  if (vVel.z < 0.0f) vVel.z = vVel.z + 0.01f;						\
			  if (vVel.z > 0.0f) vVel.z = vVel.z - 0.01f;						\
			  //																\
			  if (abs(vVel.x) < 0.02) vVel.x = 0.0f;							\
			} 																	\
          }																		\
		}																		\
																				\
        if (vPos.z < -dgWorldSize.z) {											\
            vPos.z = -dgWorldSize.z;											\
            vVel.z = -vVel.z;													\
        }																		\
																				\
        if (vPos.z > dgWorldSize.z) {											\
            vPos.z = dgWorldSize.z;												\
            vVel.z = -vVel.z;													\
        }																		\
																				\
    }																			\
  //																			\
  // Write back																	\
  Positions[index] = vec4(vPos, 1.0f);											\
  Velocities[index] = vec4(vVel, 1.0f);											\
  BodyDataA[index] = vBodA;														\
}																				\
";


