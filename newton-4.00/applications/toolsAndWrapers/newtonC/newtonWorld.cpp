/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "newtonStdafx.h"
#include "newtonWorld.h"

class ndContactCallback : public ndContactNotify
{
	public:
	enum ndMaterialUserIDs
	{
		m_default = 0,
		m_dedris = 100,
	};

	enum ndMaterialFlags
	{
		m_playSound = 1 << 0,
		//m_debrisBody = 1 << 1,
	};

	class ndMaterailKey
	{
		public:
		ndMaterailKey()
			:m_key(0)
		{
		}

		ndMaterailKey(ndUnsigned64 low, ndUnsigned64 high)
			:m_lowKey(ndUnsigned32(ndMin(low, high)))
			,m_highKey(ndUnsigned32(ndMax(low, high)))
		{
		}

		bool operator<(const ndMaterailKey& other) const
		{
			return (m_key < other.m_key);
		}

		bool operator>(const ndMaterailKey& other) const
		{
			return (m_key > other.m_key);
		}

		union
		{
			struct
			{
				ndUnsigned32 m_lowKey;
				ndUnsigned32 m_highKey;
			};
			ndUnsigned64 m_key;
		};
	};

	ndContactCallback(ndScene* const scene)
		:ndContactNotify(scene)
		//,m_materialMap()
	{
		//m_materialMap.Insert(ndMaterial(), ndMaterailKey(0, 0));
	}


	//virtual ndMaterial& RegisterMaterial(ndUnsigned32 id0, ndUnsigned32 id1);
	//
	//virtual void OnBodyAdded(ndBodyKinematic* const body) const;
	//virtual void OnBodyRemoved(ndBodyKinematic* const body) const;
	//virtual ndMaterial GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const;
	//virtual bool OnAabbOverlap(const ndContact* const contactJoint, ndFloat32 timestep);
	//virtual void OnContactCallback(ndInt32 threadIndex, const ndContact* const contactJoint, ndFloat32 timestep);
	//
	//void PlaySoundTest(const ndContact* const contactJoint);
	//
	//dTree<ndMaterial, ndMaterailKey> m_materialMap;
};

NewtonWorld::NewtonWorld()
	:ndWorld()
	,m_nominalTimestep(ndFloat32 (1.0f) / NOMINAL_FPS)
{
	ClearCache();
	SetContactNotify(new ndContactCallback(GetScene()));
}

NewtonWorld::~NewtonWorld()
{
}

void NewtonWorld::SetSubSteps(ndFloat32 timestep)
{
	timestep = ndClamp(timestep, ndFloat32(1.0f/120.0f), ndFloat32(1.0f / 24.0f)) - 0.001f;
	ndInt32 substeps = ndInt32 (ndFloor(timestep / m_nominalTimestep)) + 1;
	ndWorld::SetSubSteps(substeps);
}

void NewtonWorld::SetTimestep(ndFloat32 nominalTimestep)
{
	m_nominalTimestep = ndClamp(nominalTimestep, ndFloat32(60.0f), ndFloat32(600.0f));
}

void NewtonWorld::SetIterations(ndInt32 iterations)
{
	iterations = ndClamp(iterations, 4, 16);
	SetSolverIterations(iterations);
}

void NewtonWorld::Update(ndFloat32 timestep)
{
	timestep = ndClamp(timestep, ndFloat32(1.0f / 120.0f), ndFloat32(1.0f / 24.0f));
	ndWorld::Update(timestep);
}