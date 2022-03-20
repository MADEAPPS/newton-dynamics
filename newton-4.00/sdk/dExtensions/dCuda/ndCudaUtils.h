/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_CUDA_UTILS_H__
#define __ND_CUDA_UTILS_H__

#include <cuda.h>
#include <cuda_runtime.h>
#include <ndNewtonStdafx.h>
#include <device_launch_parameters.h>

struct matrix3x3
{
	float3 m_rows[3];
};

inline float4 VectorToFloat4(const ndVector& v)
{
	float4 data{ v.m_x, v.m_y, v.m_z, v.m_w };
	return data;
}

inline struct matrix3x3 dQuatToMatrix(float4 q)
{
	struct matrix3x3 matrix;

	//const dQuaternion quat0(quat);
	//const dQuaternion quat1(quat0.Scale (ndFloat32(2.0f)));
	float4 quat0 = q;
	//float4 quat1 = q * ((float4)(2.0f));
	float4 quat1 = { 2.0f * q.x, 2.0f * q.y, 2.0f * q.z, 2.0f * q.w };

	float x2 = quat0.x * quat1.x;
	float y2 = quat0.y * quat1.y;
	float z2 = quat0.z * quat1.z;

	float xy = quat0.x * quat1.y;
	float xz = quat0.x * quat1.z;
	float xw = quat0.x * quat1.w;
	float yz = quat0.y * quat1.z;
	float yw = quat0.y * quat1.w;
	float zw = quat0.z * quat1.w;

	//matrix.m_front = (float4)(1.0f - y2 - z2, xy + zw, xz - yw, 0.0f);
	//matrix.m_up = (float4)(xy - zw, 1.0f - x2 - z2, yz + xw, 0.0f);
	//matrix.m_right = (float4)(xz + yw, yz - xw, 1.0f - x2 - y2, 0.0f);

	return matrix;
}


#endif