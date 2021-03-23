/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

struct ndOpenclMatrix3x3
{
	float3 m_element[3];
};

struct ndOpenclBodyProxy
{
	float4 m_rotation;
	float3 m_position;
	float3 m_veloc;
	float3 m_omega;
	float4 m_invMass;
};

//struct ndOpenclOutBodyProxy
//{
//	float4 m_rotation;
//	float3 m_position;
//	float3 m_veloc;
//	float3 m_omega;
//};

struct ndOpenclBodyWorkingBuffer
{
	struct ndOpenclMatrix3x3 m_matrix;
	float4 m_rotation;
	float3 m_position;
	float3 m_veloc;
	float3 m_omega;
};

struct ndOpenclMatrix3x3 QuatToMatrix(float4 rotation)
{
	struct ndOpenclMatrix3x3 matrix;
	return matrix;
}

__kernel void IntegrateUnconstrainedBodies(float timestep, int bodyCount, __global struct ndOpenclBodyProxy* inputArray, __global struct ndOpenclBodyWorkingBuffer* outputArray)
{
	const int index = get_global_id(0);

	struct ndOpenclBodyProxy body; 

	// load all variable into registers.
	if (index < bodyCount)
	{
		body = inputArray[index];	
	}
	barrier (CLK_LOCAL_MEM_FENCE);

	if (index < bodyCount)
	{
		struct ndOpenclMatrix3x3 matrix;
		matrix = QuatToMatrix(body.m_rotation);
	}
}

