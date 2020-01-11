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


// This is an example of an exported function.
dgWorldPlugin* GetPlugin(dgWorld* const world, dgMemoryAllocator* const allocator)
{
#ifdef _WIN32
	union cpuInfo
	{
		int m_data[4];
		struct
		{
			int m_eax;
			int m_ebx;
			int m_ecx;
			int m_edx;
		};
	} info;

	__cpuid(info.m_data, 0);
	if (info.m_eax < 7) {
		return NULL;
	}

	// check for instruction set support (avx2 is bit 5 in reg ebx)
	__cpuid(info.m_data, 7);
	if (!(info.m_ebx & (1 << 5))) {
		return NULL;
	}

	static dgWorldBase module(world, allocator);
	union {
		char m_vendor[256];
		int m_reg[3];
	};
	memset(m_vendor, 0, sizeof(m_vendor));
	__cpuid(info.m_data, 0);

	m_reg[0] = info.m_ebx;
	m_reg[1] = info.m_edx;
	m_reg[2] = info.m_ecx;
	module.m_score = _stricmp(m_vendor, "GenuineIntel") ? 3 : 4;

#ifdef _DEBUG
	sprintf(module.m_hardwareDeviceName, "Newton avx2_d");
#else
	sprintf(module.m_hardwareDeviceName, "Newton avx2");
#endif
	return &module;
#elif __linux__
	if(__builtin_cpu_supports("avx2")) {
		static dgWorldBase module(world, allocator);
		module.m_score = 4;
#ifdef _DEBUG
		sprintf(module.m_hardwareDeviceName, "Newton avx2_d");
#else
		sprintf(module.m_hardwareDeviceName, "Newton avx2");
#endif
		return &module;
	} else {
		return NULL;
	}
#elif defined (_MACOSX_VER)
	// must macs support avx2 but for now let use do avx only
	//static dgWorldBase module(world, allocator);
	//module.m_score = 4;
	//return &module;
	return NULL;
#endif
}

dgWorldBase::dgWorldBase(dgWorld* const world, dgMemoryAllocator* const allocator)
	:dgWorldPlugin(world, allocator)
	,dgSolver(world, allocator)
{
}

dgWorldBase::~dgWorldBase()
{
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

/*
static DG_INLINE dgFloat32 ScalarGetMax(dgFloat32 a, dgFloat32 b)
{
#ifdef _NEWTON_USE_DOUBLE
	return  _mm_cvtsd_f64(_mm_max_sd(_mm_set_sd(a), _mm_set_sd(b)));
#else
	return  _mm_cvtss_f32(_mm_max_ss(_mm_set_ss(a), _mm_set_ss(b)));
#endif
}

static DG_INLINE dgFloat32 ScalarGetMin(dgFloat32 a, dgFloat32 b)
{
#ifdef _NEWTON_USE_DOUBLE
	return  _mm_cvtsd_f64(_mm_min_sd(_mm_set_sd(a), _mm_set_sd(b)));
#else
	return  _mm_cvtss_f32(_mm_min_ss(_mm_set_ss(a), _mm_set_ss(b)));
#endif
}

static DG_INLINE dgFloat32 ScalarPredicateResidual(dgFloat32 r, dgFloat32 f, dgFloat32 max, dgFloat32 min)
{
#ifdef _NEWTON_USE_DOUBLE
	__m128d f1(_mm_set_sd(f));
	__m128d mask0(_mm_cmplt_sd(f1, _mm_set_sd(max)));
	__m128d mask1(_mm_cmpgt_sd(f1, _mm_set_sd(min)));
	return _mm_cvtsd_f64(_mm_and_pd(_mm_and_pd(_mm_set_sd(r), mask0), mask1));
#else
	__m128 mask0(_mm_cmplt_ss(_mm_set_ss(f), _mm_set_ss(max)));
	__m128 mask1(_mm_cmpgt_ss(_mm_set_ss(f), _mm_set_ss(min)));
	return _mm_cvtss_f32(_mm_and_ps(_mm_and_ps(_mm_set_ss(r), mask0), mask1));
#endif
}
*/

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
		//x[i] = ScalarGetMax(ScalarGetMin(dgFloat32(0.0f), h), l);
		x[i] = dgClamp(dgFloat32 (0.0f), l, h);
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
#if 0
			dgFloat32 f = x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor;
			tolerance += r * ScalarPredicateResidual(r, f, h, l);
			f = ScalarGetMax(ScalarGetMin(f, h), l);
			const dgFloat32 dx = f - x[i];
#else
			//const dgFloat32 f = ScalarGetMax(ScalarGetMin(x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor, h), l);
			const dgFloat32 f = dgClamp(x[i] + ((r + row[i] * x[i]) * invDiag[i] - x[i]) * sor, l, h);
			const dgFloat32 dx = f - x[i];
			const dgFloat32 dr = dx * row[i];
			tolerance += dr * dr;
#endif 
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