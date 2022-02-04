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
#ifndef __CONTACT_CALLBACK_H__
#define __CONTACT_CALLBACK_H__

#include "ndSandboxStdafx.h"

class ndApplicationMaterial : public ndMaterial
{
	public:
	enum ndMaterialUserIDs
	{
		m_default = 0,
		m_aiCar = 1,
		m_aiTerrain = 2,
		m_dedris = 100,
	};

	enum ndMaterialFlags
	{
		m_playSound = 1 << 0,
		//m_debrisBody = 1 << 1,
	};

	ndApplicationMaterial();
	ndApplicationMaterial(const ndApplicationMaterial& src);

	virtual ndApplicationMaterial* Clone(const ndApplicationMaterial& src) const
	{
		return new ndApplicationMaterial(src);
	}

	virtual bool OnAabbOverlap(const ndContact* const joint, ndFloat32 timestep) const;
	virtual void OnContactCallback(const ndContact* const joint, ndFloat32 timestep) const;
};

class ndMaterailKey
{
	public:
	ndMaterailKey()
		:m_key(0)
	{
	}

	ndMaterailKey(ndUnsigned64 low, ndUnsigned64 high)
		:m_lowKey(ndUnsigned32(dMin(low, high)))
		, m_highKey(ndUnsigned32(dMax(low, high)))
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

class ndMaterialGraph : public ndTree<ndApplicationMaterial*, ndMaterailKey>
{
	public:
	ndMaterialGraph();
	~ndMaterialGraph();
};

class ndContactCallback: public ndContactNotify
{
	public: 
	ndContactCallback();

	virtual ndApplicationMaterial& RegisterMaterial(const ndApplicationMaterial& material, ndUnsigned32 id0, ndUnsigned32 id1);

	virtual void OnBodyAdded(ndBodyKinematic* const body) const;
	virtual void OnBodyRemoved(ndBodyKinematic* const body) const;

	virtual bool OnAabbOverlap(const ndContact* const contactJoint, ndFloat32 timestep);
	virtual void OnContactCallback(const ndContact* const contactJoint, ndFloat32 timestep);
	virtual bool OnCompoundSubShapeOverlap(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1);

	virtual ndMaterial* GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const;

	void PlaySoundTest(const ndContact* const contactJoint);
	
	ndMaterialGraph m_materialGraph;
	ndApplicationMaterial m_defaultMaterial;
};


#endif