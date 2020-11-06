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
point += dVector(0.25f);
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
	//m_box0 = box0 - dVector(m_radius);
	//m_box1 = box1 + dVector(m_radius);
	m_box0 = box0;
	m_box1 = box1;
}

union ndGridHash
{
	ndGridHash(const dVector& mask, dInt32 cellType)
		:m_posit((mask.Floor()).GetInt())
	{
		m_homeCell = cellType;
	}

	ndGridHash(const ndGridHash& src)
		:m_posit(src.m_posit)
	{
	}

	struct 
	{
		dInt32 m_x;
		dInt32 m_y;
		dInt32 m_z;
		union
		{
			dInt32 m_mask;
			struct 
			{
				dInt32 m_id : 22;
				dInt32 m_notUsed : 9;
				dInt32 m_homeCell : 1;
			};
		};
	};
	dVector m_posit;
};

static dInt32 Compare(const ndGridHash* hashA, const ndGridHash* const hashB, void* const context)
{
	if (hashA->m_x > hashB->m_x)
	{
		return 1;
	}
	else if (hashA->m_x < hashB->m_x)
	{
		return -1;
	}
	else if (hashA->m_y > hashB->m_y)
	{
		return 1;
	}
	else if (hashA->m_y < hashB->m_y)
	{
		return -1;
	}
	else if (hashA->m_z > hashB->m_z)
	{
		return 1;
	}
	else if (hashA->m_z < hashB->m_z)
	{
		return -1;
	}
	else
	{
		return 0;
	}

	dAssert(0);
}

void ndBodySphFluid::Update(dFloat32 timestep)
{
	static dArray<ndGridHash> hashGridMap;
	hashGridMap.SetCount(m_posit.GetCount() * 8);

	const dFloat32 diameter = m_radius * dFloat32(2.0f);
	const dFloat32 diameterPadd = diameter * dFloat32(1.0625f);
	const dVector invDiameter(dFloat32(1.0f) / diameter);

	UpdateAABB();
dTree<dInt32, dInt64> xxxx;

	dVector neighborkDirs[8];
	neighborkDirs[0] = dVector(-m_radius, -m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[1] = dVector( m_radius, -m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[2] = dVector(-m_radius,  m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[3] = dVector( m_radius,  m_radius, -m_radius, dFloat32(0.0f));
	neighborkDirs[4] = dVector(-m_radius, -m_radius,  m_radius, dFloat32(0.0f));
	neighborkDirs[5] = dVector( m_radius, -m_radius,  m_radius, dFloat32(0.0f));
	neighborkDirs[6] = dVector(-m_radius,  m_radius,  m_radius, dFloat32(0.0f));
	neighborkDirs[7] = dVector( m_radius,  m_radius,  m_radius, dFloat32(0.0f));

	dInt32 count = 0;
	dVector neighboarghHatch(dVector::m_zero);
	for (dInt32 i = 0; i < m_posit.GetCount(); i++)
	{
		neighboarghHatch.m_w = dFloat32(i);
		dVector r(m_posit[i] - m_box0);
		dVector p(r * invDiameter + neighboarghHatch);

		ndGridHash hashKey(p, 1);
		hashGridMap[count] = hashKey;
		count++;

		dInt64 xxxx1 = (dInt64) hashKey.m_posit.m_z * 100000 + (dInt64)hashKey.m_posit.m_y * 1000 + (dInt64)hashKey.m_posit.m_x;
		xxxx.Insert(count, xxxx1);

		hashKey.m_homeCell = 0;
		for (dInt32 j = 0; j < sizeof(neighborkDirs) / sizeof(neighborkDirs[0]); j++)
		{
			ndGridHash neighborKey(p + neighborkDirs[j], 0);
			dVector test(neighborKey.m_posit == hashKey.m_posit);
			dInt32 encroaching = test.GetSignMask() & 0x07;
			if (encroaching != 0x07)
			{
				hashGridMap[count] = neighborKey;
				count++;
			}
		}
	}

	//dSort(&hashGridMap[0], count, Compare);

}