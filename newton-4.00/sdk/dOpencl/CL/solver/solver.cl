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
	float3 m_elements[3];
};

struct ndOpenclMatrix3x4
{
	float4 m_elements[3];
};

struct ndOpenclBodyProxy
{
	struct ndOpenclMatrix3x4 m_matrix;
	float4 m_invMass;
};

struct ndOpenclInternalBodyProxy
{
	struct ndOpenclMatrix3x3 m_invWorldInertiaMatrix;
};

//__kernel void TestKernel(__global int* pA, __global int* pB, __global int* pC)
//{
//    const int x = get_global_id(0);
//    const int y = get_global_id(1);
//    const int width = get_global_size(0);
//    const int id = y * width + x;
//
//    pC[id] = pA[id] + pB[id];
//}

void UpdateInvInertiaMatrix(__global struct ndOpenclBodyProxy* inputArray, __global struct ndOpenclInternalBodyProxy* outputBuffer)
{
}

__kernel void IntegrateUnconstrainedBodies(__global struct ndOpenclBodyProxy* inputArray, __global struct ndOpenclInternalBodyProxy* outputBuffer)
{
	UpdateInvInertiaMatrix(inputArray, outputBuffer);
}
