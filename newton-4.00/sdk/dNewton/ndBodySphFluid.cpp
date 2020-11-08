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
	,m_hashGridMap()
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

void ndBodySphFluid::SortBuckets()
{
	const dInt32 count = m_hashGridMap.GetCount();
	m_sortGridMapBuffer.SetCount(count);

	dInt32 histogram0[1<<11];
	dInt32 histogram1[5][1<<10];
	memset(histogram0, 0, sizeof(histogram0));
	memset(histogram1, 0, sizeof(histogram1));

	ndGridHash* const hashArray = &m_hashGridMap[0];
	for (dInt32 i = 0; i < count; i++)
	{
		const ndGridHash& entry = hashArray[i];

		const dInt32 xlow = dInt32 (entry.m_xLow * 2 + entry.m_cellType);
		histogram0[xlow] = histogram0[xlow] + 1;

		const dInt32 xHigh = entry.m_xHigh;
		histogram1[0][xHigh] = histogram1[0][xHigh] + 1;

		const dInt32 ylow = entry.m_yLow;
		histogram1[1][ylow] = histogram1[1][ylow] + 1;

		const dInt32 yHigh = entry.m_yHigh;
		histogram1[2][yHigh] = histogram1[2][yHigh] + 1;

		const dInt32 zlow = entry.m_zLow;
		histogram1[3][zlow] = histogram1[3][zlow] + 1;

		const dInt32 zHigh = entry.m_zHigh;
		histogram1[4][zHigh] = histogram1[4][zHigh] + 1;
	}
	
	dInt32 acc0 = 0;
	for (dInt32 i = 0; i < (1 << 11); i++)
	{
		const dInt32 n = histogram0[i];
		histogram0[i] = acc0;
		acc0 += n;
	}

	dInt32 acc[5];
	memset(acc, 0, sizeof(acc));
	for (dInt32 i = 0; i < (1 << 10); i++)
	{
		for (dInt32 j = 0; j < 5; j++)
		{
			const dInt32 n = histogram1[j][i];
			histogram1[j][i] = acc[j];
			acc[j] += n;
		}
	}

	dInt32 shiftbits = 0;
	dUnsigned64 mask = ~dUnsigned64(dInt64(-1 << 10));
	ndGridHash* const tmpArray = &m_sortGridMapBuffer[0];

	for (dInt32 i = 0; i < count; i++)
	{
		const ndGridHash& entry = hashArray[i];
		const dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits) * 2 + entry.m_cellType;
		const dInt32 index = histogram0[key];
		tmpArray[index] = entry;
		histogram0[key] = index + 1;
	}
	mask <<= 10;
	shiftbits += 10;
	
	dInt32* const scan2 = &histogram1[0][0];
	for (dInt32 i = 0; i < count; i++)
	{
		const ndGridHash& entry = tmpArray[i];
		const dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
		const dInt32 index = scan2[key];
		hashArray[index] = entry;
		scan2[key] = index + 1;
	}
	mask <<= 10;
	shiftbits += 10;

	for (dInt32 radix = 0; radix < 2; radix ++)
	{
		dInt32* const scan0 = &histogram1[radix * 2 + 1][0];
		for (dInt32 i = 0; i < count; i++)
		{
			const ndGridHash& entry = hashArray[i];
			const dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
			const dInt32 index = scan0[key];
			tmpArray[index] = entry;
			scan0[key] = index + 1;
		}
		mask <<= 10;
		shiftbits += 10;
		
		dInt32* const scan1 = &histogram1[radix * 2 + 2][0];
		for (dInt32 i = 0; i < count; i++)
		{
			const ndGridHash& entry = tmpArray[i];
			const dInt32 key = dUnsigned32((entry.m_gridHash & mask) >> shiftbits);
			const dInt32 index = scan1[key];
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
		dUnsigned64 gridHashA = entry0.m_gridHash * 2 + entry0.m_cellType;
		dUnsigned64 gridHashB = entry1.m_gridHash * 2 + entry1.m_cellType;

		dAssert(gridHashA <= gridHashB);
	}
	#endif
}

void ndBodySphFluid::Update(const ndWorld* const workd, dFloat32 timestep)
{
/*
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

	if (m_hashGridMap.GetCapacity() < m_posit.GetCount() * 16)
	{
		m_hashGridMap.SetCount(m_posit.GetCount() * 16);
	}

	dInt32 count = 0;
	for (dInt32 i = 0; i < m_posit.GetCount(); i++)
	{
		dVector r(m_posit[i] - m_box0);
		dVector p(r * invGridSize);

		ndGridHash hashKey(p, i, ndHomeGrid);
		m_hashGridMap[count] = hashKey;
		count++;

		for (dInt32 j = 0; j < sizeof(neighborkDirs) / sizeof(neighborkDirs[0]); j++)
		{
			ndGridHash neighborKey(p + neighborkDirs[j], i, ndAdjacentGrid);
			if (neighborKey.m_gridHash != hashKey.m_gridHash)
			{
				m_hashGridMap[count] = neighborKey;
				count++;
			}
		}
	}
	m_hashGridMap.SetCount(count);
*/
	UpdateAABB();
	CreateGrids(workd);
	SortBuckets();
}

void ndBodySphFluid::CreateGrids(const ndWorld* const workd)
{
	const dFloat32 diameter = m_radius * dFloat32(2.0f);
	const dFloat32 gridSize = diameter * dFloat32(1.0625f);
	const dVector invGridSize(dFloat32(1.0f) / gridSize);

	dVector neighborkDirs[8];
	neighborkDirs[0] = dVector(-m_radius, -m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[1] = dVector(m_radius, -m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[2] = dVector(-m_radius, m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[3] = dVector(m_radius, m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[4] = dVector(-m_radius, -m_radius, m_radius, dFloat32(0.0f));
	neighborkDirs[5] = dVector(m_radius, -m_radius, m_radius, dFloat32(0.0f));
	neighborkDirs[6] = dVector(-m_radius, m_radius, m_radius, dFloat32(0.0f));
	neighborkDirs[7] = dVector(m_radius, m_radius, m_radius, dFloat32(0.0f));

	if (m_hashGridMap.GetCapacity() < m_posit.GetCount() * 16)
	{
		m_hashGridMap.SetCount(m_posit.GetCount() * 16);
	}

	dInt32 count = 0;
	for (dInt32 i = 0; i < m_posit.GetCount(); i++)
	{
		dVector r(m_posit[i] - m_box0);
		dVector p(r * invGridSize);

		ndGridHash hashKey(p, i, ndHomeGrid);
		m_hashGridMap[count] = hashKey;
		count++;

		for (dInt32 j = 0; j < sizeof(neighborkDirs) / sizeof(neighborkDirs[0]); j++)
		{
			ndGridHash neighborKey(p + neighborkDirs[j], i, ndAdjacentGrid);
			if (neighborKey.m_gridHash != hashKey.m_gridHash)
			{
				m_hashGridMap[count] = neighborKey;
				count++;
			}
		}
	}
	m_hashGridMap.SetCount(count);
}