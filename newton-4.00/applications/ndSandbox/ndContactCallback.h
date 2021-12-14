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

class ndContactCallback: public ndContactNotify
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
			:m_lowKey(ndUnsigned32(dMin(low, high)))
			,m_highKey(ndUnsigned32(dMax(low, high)))
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

	ndContactCallback();

	virtual ndMaterial& RegisterMaterial(ndUnsigned32 id0, ndUnsigned32 id1);

	virtual void OnBodyAdded(ndBodyKinematic* const body) const;
	virtual void OnBodyRemoved(ndBodyKinematic* const body) const;
	virtual ndMaterial GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const;
	virtual bool OnAabbOverlap(const ndContact* const contactJoint, ndFloat32 timestep);
	virtual void OnContactCallback(ndInt32 threadIndex, const ndContact* const contactJoint, ndFloat32 timestep);

	void PlaySoundTest(const ndContact* const contactJoint);

	ndTree<ndMaterial, ndMaterailKey> m_materialMap;
};


#endif