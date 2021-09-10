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

		ndMaterailKey(dUnsigned64 low, dUnsigned64 high)
			:m_lowKey(dUnsigned32(dMin(low, high)))
			, m_highKey(dUnsigned32(dMax(low, high)))
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
				dUnsigned32 m_lowKey;
				dUnsigned32 m_highKey;
			};
			dUnsigned64 m_key;
		};
	};

	ndContactCallback::ndContactCallback()
		:ndContactNotify()
		//,m_materialMap()
	{
		//m_materialMap.Insert(ndMaterial(), ndMaterailKey(0, 0));
	}


	//virtual ndMaterial& RegisterMaterial(dUnsigned32 id0, dUnsigned32 id1);
	//
	//virtual void OnBodyAdded(ndBodyKinematic* const body) const;
	//virtual void OnBodyRemoved(ndBodyKinematic* const body) const;
	//virtual ndMaterial GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const;
	//virtual bool OnAabbOverlap(const ndContact* const contactJoint, dFloat32 timestep);
	//virtual void OnContactCallback(dInt32 threadIndex, const ndContact* const contactJoint, dFloat32 timestep);
	//
	//void PlaySoundTest(const ndContact* const contactJoint);
	//
	//dTree<ndMaterial, ndMaterailKey> m_materialMap;
};

NewtonWorld::NewtonWorld()
	:ndWorld()
	,m_nominalTimestep(dFloat32 (1.0f) / NOMINAL_FPS)
{
	ClearCache();
	SetContactNotify(new ndContactCallback);
}

NewtonWorld::~NewtonWorld()
{
}

void NewtonWorld::SetSubSteps(dFloat32 timestep)
{
	timestep = dClamp(timestep, dFloat32(1.0f/120.0f), dFloat32(1.0f / 24.0f)) - 0.001f;
	dInt32 substeps = dInt32 (dFloor(timestep / m_nominalTimestep)) + 1;
	ndWorld::SetSubSteps(substeps);
}

void NewtonWorld::SetTimestep(dFloat32 nominalTimestep)
{
	m_nominalTimestep = dClamp(nominalTimestep, dFloat32(60.0f), dFloat32(600.0f));
}

void NewtonWorld::SetIterations(dInt32 iterations)
{
	iterations = dClamp(iterations, 4, 16);
	SetSolverIterations(iterations);
}

void NewtonWorld::Update(dFloat32 timestep)
{
	timestep = dClamp(timestep, dFloat32(1.0f / 120.0f), dFloat32(1.0f / 24.0f));
	ndWorld::Update(timestep);
}