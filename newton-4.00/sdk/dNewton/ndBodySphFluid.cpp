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
	,m_box1(dFloat32(-1e10f))
{
//	dAssert(0);
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

#ifndef D_USE_IN_PLACE_BUCKETS
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
	dSort(&hashArray[0], count, Compare);
}
#endif

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
	
#ifdef D_USE_IN_PLACE_BUCKETS
	ndCellMap uniqueGridId;
	for (dInt32 i = 0; i < m_posit.GetCount(); i++)
	{
		dVector r(m_posit[i] - m_box0);
		dVector p(r * invGridSize);
		ndGridHash hashKey(p, i, ndHomeGrid);
		uniqueGridId.AddGrid(hashKey);

		for (dInt32 j = 0; j < sizeof(neighborkDirs) / sizeof(neighborkDirs[0]); j++)
		{
			ndGridHash neighborKey(p + neighborkDirs[j], i, ndAdjacentGrid);
			if (neighborKey.m_gridHash != hashKey.m_gridHash)
			{
				uniqueGridId.AddGrid(neighborKey);
			}
		}
	}

#else

	static dArray<ndGridHash> hashGridMap;
	hashGridMap.SetCount(m_posit.GetCount() * 8);

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
#endif

}