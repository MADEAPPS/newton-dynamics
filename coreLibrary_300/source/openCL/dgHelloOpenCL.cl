/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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

union XXX
{
	float16 a;
	float8 a0[2];
	float4 a1[4];
	float2 a2[8];
	float a3[16];
};

__kernel void __attribute__((vec_type_hint(float16))) dgHelloOpenCl (__global const float16* const A, __global const float16* const B, __global float* const out)
{
	int idx = get_global_id (0);
//	out[idx] = A[idx] + B[idx];
	union XXX xxx;
	xxx.a = A[idx] + B[idx];
	xxx.a0[0] += xxx.a0[1];
	xxx.a1[0] += xxx.a1[1];
	xxx.a2[0] += xxx.a2[1];
	out[idx] = xxx.a3[0] + xxx.a3[1]; 
}