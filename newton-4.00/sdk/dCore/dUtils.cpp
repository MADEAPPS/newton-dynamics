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

#include "dCoreStdafx.h"
#include "dTypes.h"
#include "dUtils.h"
#include "dVector.h"
#include "dMatrix.h"

dFloat64 dRoundToFloat(dFloat64 val)
{
	dInt32 exp;
	dFloat64 mantissa = frexp(val, &exp);

	const dFloat64 power = 1 << 23;
	const dFloat64 invPower = dFloat64(1.0f) / power;
	mantissa = floor(mantissa * power) * invPower;

	dFloat64 val1 = ldexp(mantissa, exp);
	return val1;
}

dUnsigned64 dGetTimeInMicrosenconds()
{
	static std::chrono::high_resolution_clock::time_point timeStampBase = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point currentTimeStamp = std::chrono::high_resolution_clock::now();
	dUnsigned64 timeStamp = std::chrono::duration_cast<std::chrono::microseconds>(currentTimeStamp - timeStampBase).count();
	return timeStamp;
}

dFloatExceptions::dFloatExceptions(dUnsigned32 mask)
{
#if defined (_MSC_VER)
	dClearFP();
	m_mask = dControlFP(0, 0);
	dControlFP(m_mask & ~mask, _MCW_EM);
#endif

#if defined (__APPLE__)
	#ifndef IOS
		fesetenv(FE_DFL_DISABLE_SSE_DENORMS_ENV);
	#endif
#elif defined (WIN32)
	_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	dInt32 crs = _mm_getcsr();
	dInt32 sseDenormalMask = _MM_FLUSH_ZERO_MASK | _MM_MASK_DENORM;
	_mm_setcsr(crs | sseDenormalMask);
#elif (defined (_M_ARM) || defined (_M_ARM64))
	//_MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
	#pragma message ("warning!!! do not forget to set flush to zero for arm cpus")
#endif

	//dFloat32 a = dFloat32(1.0f);
	//dFloat32 b = dFloat32(0.5f);
	//dInt32 count = 0;
	//while (a != 0.0f)
	//{
	//	a = a * b;
	//	count++;
	//}
	//count++;
}

dFloatExceptions::~dFloatExceptions()
{
#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	dClearFP();
	dControlFP(m_mask, _MCW_EM);
#endif
}

dSetPrecisionDouble::dSetPrecisionDouble()
{
#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	dClearFP();
	m_mask = dInt32(dControlFP(0, 0));
	dControlFP(_PC_53, _MCW_PC);
#endif
}

dSetPrecisionDouble::~dSetPrecisionDouble()
{
#if (defined (_MSC_VER) && defined (_WIN_32_VER))
	dClearFP();
	dControlFP(dUnsigned32(m_mask), _MCW_PC);
#endif
}

static inline dInt32 cmp_vertex(const dFloat64* const v1, const dFloat64* const v2, dInt32 firstSortAxis)
{
	if (v1[firstSortAxis] < v2[firstSortAxis]) 
	{
		return -1;
	}

	if (v1[firstSortAxis] > v2[firstSortAxis]) 
	{
		return 1;
	}

	return 0;
}

static dInt32 SortVertices(dFloat64* const vertexList, dInt32 stride, dInt32 compareCount, dInt32 vertexCount, dFloat64 tolerance)
{
	dBigVector xc(dBigVector::m_zero);
	dBigVector x2c(dBigVector::m_zero);
	dBigVector minP(dFloat64(1.0e10f), dFloat64(1.0e10f), dFloat64(1.0e10f), dFloat64(0.0f));
	dBigVector maxP(dFloat64(-1.0e10f), dFloat64(-1.0e10f), dFloat64(-1.0e10f), dFloat64(0.0f));

	for (dInt32 k = 0, i = 0; i < vertexCount; i++) 
	{
		dBigVector x(vertexList[k + 2], vertexList[k + 3], vertexList[k + 4], dFloat64(0.0f));
		xc += x;
		x2c += x * x;
		minP = minP.GetMin(x);
		maxP = maxP.GetMax(x);
		k += stride;
	}

	dBigVector del(maxP - minP);
	dFloat64 minDist = dMin(del.m_x, del.m_y, del.m_z);
	if (minDist < dFloat64(1.0e-3f)) 
	{
		minDist = dFloat64(1.0e-3f);
	}

	dFloat64 tol = tolerance * minDist + dFloat64(1.0e-12f);
	dFloat64 sweptWindow = dFloat64(2.0f) * tol;
	sweptWindow += dFloat64(1.0e-4f);

	x2c = x2c.Scale(dFloat32(vertexCount)) - xc * xc;

	dInt32 firstSortAxis = 2;
	if ((x2c.m_y >= x2c.m_x) && (x2c.m_y >= x2c.m_z)) 
	{
		firstSortAxis = 3;
	}
	else if ((x2c.m_z >= x2c.m_x) && (x2c.m_z >= x2c.m_y)) 
	{
		firstSortAxis = 4;
	}

	dInt32 stack[1024][2];
	stack[0][0] = 0;
	stack[0][1] = vertexCount - 1;
	dInt32 stackIndex = 1;
	while (stackIndex) 
	{
		stackIndex--;
		dInt32 lo = stack[stackIndex][0];
		dInt32 hi = stack[stackIndex][1];
		if ((hi - lo) > 8) 
		{
			dInt32 i = lo;
			dInt32 j = hi;
			dFloat64 val[64];
			memcpy(val, &vertexList[((lo + hi) >> 1) * stride], stride * sizeof(dFloat64));
			do 
			{
				while (cmp_vertex(&vertexList[i * stride], val, firstSortAxis) < 0) i++;
				while (cmp_vertex(&vertexList[j * stride], val, firstSortAxis) > 0) j--;

				if (i <= j) 
				{
					if (i < j) 
					{
						dFloat64 tmp[64];
						memcpy(tmp, &vertexList[i * stride], stride * sizeof(dFloat64));
						memcpy(&vertexList[i * stride], &vertexList[j * stride], stride * sizeof(dFloat64));
						memcpy(&vertexList[j * stride], tmp, stride * sizeof(dFloat64));
					}
					i++;
					j--;
				}
			} while (i <= j);

			if (i < hi)
			{
				stack[stackIndex][0] = i;
				stack[stackIndex][1] = hi;
				stackIndex++;
			}
			if (lo < j) 
			{
				stack[stackIndex][0] = lo;
				stack[stackIndex][1] = j;
				stackIndex++;
			}
			dAssert(stackIndex < dInt32(sizeof(stack) / (2 * sizeof(stack[0][0]))));
		}
		else 
		{
			for (dInt32 i = lo + 1; i <= hi; i++) 
			{
				dFloat64 tmp[64];
				memcpy(tmp, &vertexList[i * stride], stride * sizeof(dFloat64));

				dInt32 j = i;
				for (; j && (cmp_vertex(&vertexList[(j - 1) * stride], tmp, firstSortAxis) > 0); j--) 
				{
					memcpy(&vertexList[j * stride], &vertexList[(j - 1)* stride], stride * sizeof(dFloat64));
				}
				memcpy(&vertexList[j * stride], tmp, stride * sizeof(dFloat64));
			}
		}
	}

#ifdef _DEBUG
	for (dInt32 i = 0; i < (vertexCount - 1); i++) 
	{
		dAssert(cmp_vertex(&vertexList[i * stride], &vertexList[(i + 1) * stride], firstSortAxis) <= 0);
	}
#endif

	dInt32 count = 0;
	for (dInt32 i = 0; i < vertexCount; i++) 
	{
		dInt32 m = i * stride;
		dInt32 index = dInt32(vertexList[m + 0]);
		if (index == dInt32(0xffffffff)) 
		{
			dFloat64 swept = vertexList[m + firstSortAxis] + sweptWindow;
			dInt32 k = i * stride + stride;
			for (dInt32 i1 = i + 1; i1 < vertexCount; i1++) 
			{

				index = dInt32(vertexList[k + 0]);
				if (index == dInt32(0xffffffff)) 
				{
					dFloat64 val = vertexList[k + firstSortAxis];
					if (val >= swept) 
					{
						break;
					}
					bool test = true;
					for (dInt32 t = 0; test && (t < compareCount); t++) 
					{
						val = fabs(vertexList[m + t + 2] - vertexList[k + t + 2]);
						test = test && (val <= tol);
					}
					if (test) 
					{
						vertexList[k + 0] = dFloat64(count);
					}
				}
				k += stride;
			}

			dAssert(&vertexList[count * stride + 2] <= &vertexList[m + 2]);
			if (&vertexList[count * stride + 2] < &vertexList[m + 2]) 
			{
				memcpy(&vertexList[count * stride + 2], &vertexList[m + 2], (stride - 2) * sizeof(dFloat64));
			}
			vertexList[m + 0] = dFloat64(count);
			count++;
		}
	}
	return count;
}

static dInt32 QuickSortVertices(dFloat64* const vertList, dInt32 stride, dInt32 compareCount, dInt32 vertexCount, dFloat64 tolerance)
{
	dInt32 count = 0;
	if (vertexCount > (1024 * 256)) 
	{
		dFloat64 x = dFloat32(0.0f);
		dFloat64 y = dFloat32(0.0f);
		dFloat64 z = dFloat32(0.0f);
		dFloat64 xd = dFloat32(0.0f);
		dFloat64 yd = dFloat32(0.0f);
		dFloat64 zd = dFloat32(0.0f);

		for (dInt32 i = 0; i < vertexCount; i++) 
		{
			dFloat64 x0 = vertList[i * stride + 2];
			dFloat64 y0 = vertList[i * stride + 3];
			dFloat64 z0 = vertList[i * stride + 4];
			x += x0;
			y += y0;
			z += z0;
			xd += x0 * x0;
			yd += y0 * y0;
			zd += z0 * z0;
		}

		xd = vertexCount * xd - x * x;
		yd = vertexCount * yd - y * y;
		zd = vertexCount * zd - z * z;

		dInt32 axis = 2;
		dFloat64 axisVal = x / vertexCount;
		if ((yd > xd) && (yd > zd)) 
		{
			axis = 3;
			axisVal = y / vertexCount;
		}
		if ((zd > xd) && (zd > yd)) 
		{
			axis = 4;
			axisVal = z / vertexCount;
		}

		dInt32 i0 = 0;
		dInt32 i1 = vertexCount - 1;
		do 
		{
			for (; vertList[i0 * stride + axis] < axisVal; i0++);
			for (; vertList[i1 * stride + axis] > axisVal; i1--);
			if (i0 <= i1) 
			{
				for (dInt32 i = 0; i < stride; i++) 
				{
					dSwap(vertList[i0 * stride + i], vertList[i1 * stride + i]);
				}
				i0++;
				i1--;
			}
		} while (i0 <= i1);
		dAssert(i0 < vertexCount);

		dInt32 count0 = QuickSortVertices(&vertList[0 * stride], stride, compareCount, i0, tolerance);
		dInt32 count1 = QuickSortVertices(&vertList[i0 * stride], stride, compareCount, vertexCount - i0, tolerance);

		count = count0 + count1;

		for (dInt32 i = 0; i < count1; i++) 
		{
			memcpy(&vertList[(count0 + i) * stride + 2], &vertList[(i0 + i) * stride + 2], (stride - 2) * sizeof(dFloat64));
		}


		//dFloat64* const indexPtr = (dInt64*)vertList;
		for (dInt32 i = i0; i < vertexCount; i++) 
		{
			//indexPtr[i * stride] += count0;
			vertList[i * stride] += dFloat64(count0);
		}

	}
	else 
	{
		count = SortVertices(vertList, stride, compareCount, vertexCount, tolerance);
	}

	return count;
}

dInt32 dVertexListToIndexList(dFloat64* const vertList, dInt32 strideInBytes, dInt32 compareCount, dInt32 vertexCount, dInt32* const indexListOut, dFloat64 tolerance)
{
	dSetPrecisionDouble precision;

	if (strideInBytes < 3 * dInt32(sizeof(dFloat64))) 
	{
		return 0;
	}
	if (compareCount < 3) 
	{
		return 0;
	}
	dAssert(compareCount <= dInt32(strideInBytes / sizeof(dFloat64)));
	dAssert(strideInBytes == dInt32(sizeof(dFloat64) * (strideInBytes / sizeof(dFloat64))));

	dInt32 stride = strideInBytes / dInt32(sizeof(dFloat64));
	dInt32 stride2 = stride + 2;

	dStack<dFloat64>pool(stride2  * vertexCount);
	dFloat64* const tmpVertexList = &pool[0];

	dInt32 k = 0;
	dInt32 m = 0;
	for (dInt32 i = 0; i < vertexCount; i++) 
	{
		memcpy(&tmpVertexList[m + 2], &vertList[k], stride * sizeof(dFloat64));
		tmpVertexList[m + 0] = dFloat64(-1.0f);
		tmpVertexList[m + 1] = dFloat64(i);
		k += stride;
		m += stride2;
	}

	dInt32 count = QuickSortVertices(tmpVertexList, stride2, compareCount, vertexCount, tolerance);

	k = 0;
	m = 0;
	for (dInt32 i = 0; i < count; i++) 
	{
		k = i * stride;
		m = i * stride2;
		memcpy(&vertList[k], &tmpVertexList[m + 2], stride * sizeof(dFloat64));
		k += stride;
		m += stride2;
	}

	m = 0;
	for (dInt32 i = 0; i < vertexCount; i++) 
	{
		dInt32 i1 = dInt32(tmpVertexList[m + 1]);
		dInt32 index = dInt32(tmpVertexList[m + 0]);
		indexListOut[i1] = index;
		m += stride2;
	}
	return count;
}

#ifndef D_USE_THREAD_EMULATION
void dSpinLock::Delay(dInt32& exp)
{
	#if defined (WIN32)
		// adding exponential pause delay
		for (dInt32 i = 0; i < exp; i++)
		{
			_mm_pause();
			_mm_pause();
			//_mm_pause();
			//_mm_pause();
		}
		exp = dMin(exp * 2, 64);
	#else
		// use standard thread yield on non x86 platforms 
		//std::this_thread::yield();
		volatile dInt32 acc = 0;
		for (dInt32 i = 0; i < exp; i++)
		{
			acc++;
			acc++;
			//acc++;
			//acc++;
		}
		exp = dMin(exp * 2, 64);
	#endif
}
#endif
