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

#include "dgNewtonPluginStdafx.h"
#include "dgWorldBase.h"


char dgWorldBase::m_libPath[256];


// This is an example of an exported function.
dgWorldPlugin* GetPlugin(dgWorld* const world, dgMemoryAllocator* const allocator)
{
	VkInstance instance = dgVulkanContext::CreateInstance(allocator);
	if (instance == 0) {
		return NULL;
	}

	static dgWorldBase module(world, allocator);
	module.m_score = 10;
module.m_score = 0;
	//module.InitDevice(instance, &vkAllocators);
	dgVulkanContext& context = module.GetContext();
	context.InitDevice(instance);

#ifdef _DEBUG
	sprintf (module.m_hardwareDeviceName, "NewtonVK_d %s", context.m_gpu_props.deviceName);
#else
	sprintf(module.m_hardwareDeviceName, "NewtonVK %s", context.m_gpu_props.deviceName);
#endif

	return &module;
}

dgWorldBase::dgWorldBase(dgWorld* const world, dgMemoryAllocator* const allocator)
	:dgWorldPlugin(world, allocator)
	,dgSolver(world, allocator)
{
}

dgWorldBase::~dgWorldBase()
{
	Cleanup();
	dgVulkanContext& context = GetContext();
	context.DestroyDevice();
}

const char* dgWorldBase::GetId() const
{
	return m_hardwareDeviceName;
}

dgInt32 dgWorldBase::GetScore() const
{
	return m_score;
}

void dgWorldBase::FlushRegisters() const
{
	dgSoaFloat::FlushRegisters();
}

void dgWorldBase::CalculateJointForces(const dgBodyCluster& cluster, dgBodyInfo* const bodyArray, dgJointInfo* const jointArray, dgFloat32 timestep)
{
	dgSolver::CalculateJointForces(cluster, bodyArray, jointArray, timestep);
}


static DG_INLINE dgFloat32 MoveConditional(dgInt32 mask, dgFloat32 a, dgFloat32 b)
{
	dgAssert(mask <= 0);
#ifdef _NEWTON_USE_DOUBLE
	dgInt64 mask1 = dgInt64(mask) >> 63;
	return  _mm_cvtsd_f64(_mm_xor_pd(_mm_set_sd(b), _mm_and_pd(_mm_set_sd(*(dgFloat64*)&mask1), _mm_xor_pd(_mm_set_sd(b), _mm_set_sd(a)))));
#else
	dgInt32 mask1 = mask >> 31;
	return  _mm_cvtss_f32(_mm_xor_ps(_mm_set_ss(b), _mm_and_ps(_mm_set_ss(*(dgFloat32*)&mask1), _mm_xor_ps(_mm_set_ss(b), _mm_set_ss(a)))));
#endif
}

void dgWorldBase::SolveDenseLcp(dgInt32 stride, dgInt32 size, const dgFloat32* const matrix, const dgFloat32* const x0, dgFloat32* const x, const dgFloat32* const b, const dgFloat32* const low, const dgFloat32* const high, const dgInt32* const normalIndex) const
{
	const dgFloat32 sor = dgFloat32(1.125f);
	const dgFloat32 tol2 = dgFloat32(0.25f);
	dgFloat32* const invDiag1 = dgAlloca(dgFloat32, size);
	dgFloat32* const residual = dgAlloca(dgFloat32, size);

	dgInt32 rowStart = 0;
	const dgInt32 maxIterCount = 64;
	const dgFloat32 one = dgFloat32(1.0f);
	for (dgInt32 i = 0; i < size; i++) {
		const int index = normalIndex[i];
		const dgFloat32 coefficient = index ? (x[i + index] + x0[i + index]) : one;
		const dgFloat32 l = low[i] * coefficient - x0[i];
		const dgFloat32 h = high[i] * coefficient - x0[i];;
		x[i] = dgClamp(dgFloat32(0.0f), l, h);
		invDiag1[i] = dgFloat32(1.0f) / matrix[rowStart + i];
		rowStart += stride;
	}

	dgInt32 base = 0;
	for (dgInt32 i = 0; i < size; i++) {
		const dgFloat32* const row = &matrix[base];
		residual[i] = b[i] - dgDotProduct(size, row, x);
		base += stride;
	}

	dgInt32 iterCount = 0;
	dgFloat32 tolerance(tol2 * dgFloat32(2.0f));
	const dgFloat32* const invDiag = invDiag1;
	for (dgInt32 k = 0; (k < maxIterCount) && (tolerance > tol2); k++) {
		base = 0;
		iterCount++;
		tolerance = dgFloat32(0.0f);
		for (dgInt32 i = 0; i < size; i++) {
			const dgFloat32 r = residual[i];
			const int index = normalIndex[i];
			//const dgFloat32 x1 = x[i + index] + x0[i + index];
			//const dgFloat32 coefficient = index ? x1 : one;
			const dgFloat32 coefficient = MoveConditional(index, x[i + index] + x0[i + index], dgFloat32(1.0f));
			const dgFloat32 l = low[i] * coefficient - x0[i];
			const dgFloat32 h = high[i] * coefficient - x0[i];

			const dgFloat32* const row = &matrix[base];
			const dgFloat32 f = dgClamp(x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor, l, h);
			const dgFloat32 dx = f - x[i];
			const dgFloat32 dr = dx * row[i];
			tolerance += dr * dr;

			x[i] = f;
			if (dgAbs(dx) > dgFloat32(1.0e-6f)) {
				for (dgInt32 j = 0; j < size; j++) {
					residual[j] -= row[j] * dx;
				}
			}
			base += stride;
		}
	}
	dgSoaFloat::FlushRegisters();
}

