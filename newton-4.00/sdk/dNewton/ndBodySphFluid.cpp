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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodySphFluid.h"

ndBodySphFluid::ndBodySphFluid()
	:ndBodyParticleSet()
	,m_box0(dFloat32(-1e10f))
	,m_box1(dFloat32(1e10f))
{
}

ndBodySphFluid::ndBodySphFluid(const nd::TiXmlNode* const xmlNode, const dTree<const ndShape*, dUnsigned32>& shapesCache)
	:ndBodyParticleSet(xmlNode->FirstChild("ndBodyKinematic"), shapesCache)
{
	// nothing was saved
	dAssert(0);
}

ndBodySphFluid::~ndBodySphFluid()
{
}

void ndBodySphFluid::Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 nodeid, const dTree<dUnsigned32, const ndShape*>& shapesCache) const
{
	dAssert(0);
	nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBodySphFluid", nodeid);
	ndBodyParticleSet::Save(paramNode, assetPath, nodeid, shapesCache);
}

void ndBodySphFluid::AddParticle(const dFloat32 mass, const dVector& position, const dVector& velocity)
{
	dVector point(position);
	point.m_w = dFloat32(1.0f);
	m_posit.PushBack(point);
}

void ndBodySphFluid::UpdateAABB()
{
	dVector box0(dFloat32(1e20f));
	dVector box1(dFloat32(1e20f));
	for (dInt32 i = m_posit.GetCount() - 1; i >= 0; i--)
	{
		box0 = box0.GetMin(m_posit[i]);
		box1 = box1.GetMax(m_posit[i]);
	}
	m_box0 = box0 - dVector(m_radius * dFloat32(2.0f));
	m_box1 = box1 + dVector(m_radius * dFloat32(2.0f));
}


dInt32 ndBodySphFluid::Compare(const ndGridHash* const hashA, const ndGridHash* const hashB, void* const context)
{
	dUnsigned64 gridHashA = hashA->m_gridHash * 2 + hashA->m_cellType;
	dUnsigned64 gridHashB = hashB->m_gridHash * 2 + hashB->m_cellType;
	if (gridHashA < gridHashB)
	{
		return -1;
	}
	else if (gridHashA > gridHashB)
	{
		return 1;
	}
	return 0;
}

void ndBodySphFluid::SortBuckets(ndGridHash* const hashArray, dInt32 count)
{
#if 0
	dSort(&hashArray[0], count, Compare);
#else

	//memset(histogram, 0, sizeof(histogram));
	//for (dInt32 i = 0; i < elements; i++)
	//{
	//	dInt32 key = getRadixKey(&array[i], context);
	//	for (dInt32 j = 0; j < radixPass; j++)
	//	{
	//		dInt32 radix = (key >> (j << 3)) & 0xff;
	//		histogram[radix][j] = histogram[radix][j] + 1;
	//	}
	//}
	//
	//for (dInt32 radix = 0; radix < radixPass; radix += 2)
	//{
	//	scanCount[0] = 0;
	//	for (dInt32 i = 1; i < 256; i++)
	//	{
	//		scanCount[i] = scanCount[i - 1] + histogram[i - 1][radix];
	//	}
	//	dInt32 radixShift = radix << 3;
	//	for (dInt32 i = 0; i < elements; i++)
	//	{
	//		dInt32 key = (getRadixKey(&array[i], context) >> radixShift) & 0xff;
	//		dInt32 index = scanCount[key];
	//		tmpArray[index] = array[i];
	//		scanCount[key] = index + 1;
	//	}
	//
	//	if ((radix + 1) < radixPass)
	//	{
	//		scanCount[0] = 0;
	//		for (dInt32 i = 1; i < 256; i++) {
	//			scanCount[i] = scanCount[i - 1] + histogram[i - 1][radix + 1];
	//		}
	//
	//		dInt32 radixShift = (radix + 1) << 3;
	//		for (dInt32 i = 0; i < elements; i++)
	//		{
	//			dInt32 key = (getRadixKey(&array[i], context) >> radixShift) & 0xff;
	//			dInt32 index = scanCount[key];
	//			array[index] = tmpArray[i];
	//			scanCount[key] = index + 1;
	//		}
	//	}
	//	else
	//	{
	//		memcpy(array, tmpArray, elements * sizeof(T));
	//	}
	//}
	//
	//#ifdef _DEBUG
	//for (dInt32 i = 0; i < (elements - 1); i++)
	//{
	//	dAssert(getRadixKey(&array[i], context) <= getRadixKey(&array[i + 1], context));
	//}
	//#endif

	static dArray<ndGridHash> tmpArrayBuffer;
	tmpArrayBuffer.SetCount(count);

	dInt32 histogram[6][1<<10];
	memset(histogram, 0, sizeof(histogram));
	for (dInt32 i = 0; i < count; i++)
	{
		const ndGridHash& entry = hashArray[i];

		dInt32 xlow = entry.m_xLow;
		histogram[0][xlow] = histogram[0][xlow] + 1;

		dInt32 xHigh = entry.m_xLow;
		histogram[1][xHigh] = histogram[1][xHigh] + 1;

		dInt32 ylow = entry.m_yLow;
		histogram[2][ylow] = histogram[2][ylow] + 1;

		dInt32 yHigh = entry.m_yLow;
		histogram[3][yHigh] = histogram[3][yHigh] + 1;

		dInt32 zlow = entry.m_zLow;
		histogram[4][zlow] = histogram[4][zlow] + 1;

		dInt32 zHigh = entry.m_zLow;
		histogram[5][zHigh] = histogram[5][zHigh] + 1;
	}


	dInt32 acc[6];
	memset(acc, 0, sizeof(acc));
	for (dInt32 i = 0; i < (1 << 10); i++)
	{
		for (dInt32 j = 0; j < 6; j++)
		{
			dInt32 n = histogram[j][i];
			histogram[j][i] = acc[j];
			acc[j] += n;
		}
	}

	dInt32 shiftbits = 0;
	dUnsigned64 mask = ~dUnsigned64(dInt64(-1 << 10));
	ndGridHash* const tmpArray = &tmpArrayBuffer[0];
	for (dInt32 radix = 0; radix < 3; radix ++)
	{
		dInt32* const scan0 = &histogram[radix * 2 + 0][0];
		for (dInt32 i = 0; i < count; i++)
		{
			const ndGridHash& entry = hashArray[i];
			dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
			dInt32 index = scan0[key];
			tmpArray[index] = entry;
			scan0[key] = index + 1;
		}
		mask <<= 10;
		shiftbits += 10;

		dInt32* const scan1 = &histogram[radix * 2 + 1][0];
		for (dInt32 i = 0; i < count; i++)
		{
			const ndGridHash& entry = tmpArray[i];
			dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
			dInt32 index = scan1[key];
			hashArray[index] = entry;
			scan1[key] = index + 1;
		}
		mask <<= 10;
		shiftbits += 10;
	}
	
	#ifdef _DEBUG
	for (dInt32 i = 0; i < (count - 1); i++)
	{
		const ndGridHash& entry0 = hashArray[i + 0];
		const ndGridHash& entry1 = hashArray[i + 1];
		//dUnsigned64 gridHashA = entry0.m_gridHash * 2 + entry0.m_cellType;
		//dUnsigned64 gridHashB = entry1.m_gridHash * 2 + entry1.m_cellType;

		dUnsigned64 gridHashA = entry0.m_gridHash;
		dUnsigned64 gridHashB = entry1.m_gridHash;
		dAssert(gridHashA <= gridHashB);
	}
	#endif
#endif
}

void ndBodySphFluid::Update(dFloat32 timestep)
{
	const dFloat32 diameter = m_radius * dFloat32(2.0f);
	const dFloat32 gridSize = diameter * dFloat32(1.0625f);
	const dVector invGridSize(dFloat32(1.0f) / gridSize);

	UpdateAABB();

	dVector neighborkDirs[8];
	neighborkDirs[0] = dVector(-m_radius, -m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[1] = dVector( m_radius, -m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[2] = dVector(-m_radius,  m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[3] = dVector( m_radius,  m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[4] = dVector(-m_radius, -m_radius,  m_radius, dFloat32(0.0f));
	neighborkDirs[5] = dVector( m_radius, -m_radius,  m_radius, dFloat32(0.0f));
	neighborkDirs[6] = dVector(-m_radius,  m_radius,  m_radius, dFloat32(0.0f));
	neighborkDirs[7] = dVector( m_radius,  m_radius,  m_radius, dFloat32(0.0f));

	static dArray<ndGridHash> hashGridMap;
	hashGridMap.SetCount(m_posit.GetCount() * 16);

	dInt32 count = 0;
	for (dInt32 i = 0; i < m_posit.GetCount(); i++)
	{
		dVector r(m_posit[i] - m_box0);
		dVector p(r * invGridSize);

		ndGridHash hashKey(p, i, ndHomeGrid);
		hashGridMap[count] = hashKey;
		count++;

		for (dInt32 j = 0; j < sizeof(neighborkDirs) / sizeof(neighborkDirs[0]); j++)
		{
			ndGridHash neighborKey(p + neighborkDirs[j], i, ndAdjacentGrid);
			if (neighborKey.m_gridHash != hashKey.m_gridHash)
			{
				hashGridMap[count] = neighborKey;
				count++;
			}
		}
	}

	SortBuckets(&hashGridMap[0], count);
}