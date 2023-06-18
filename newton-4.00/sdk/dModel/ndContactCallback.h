/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
#ifndef __ND_CONTACT_CALLBACK_H__
#define __ND_CONTACT_CALLBACK_H__
		  
class ndApplicationMaterial : public ndMaterial
{
	public:
	ndApplicationMaterial();
	ndApplicationMaterial(const ndApplicationMaterial& src);
	virtual ~ndApplicationMaterial();

	virtual ndApplicationMaterial* Clone() const
	{
		return new ndApplicationMaterial(*this);
	}

	virtual void OnContactCallback(const ndContact* const joint, ndFloat32 timestep) const;
	virtual bool OnAabbOverlap(const ndContact* const joint, ndFloat32 timestep, const ndShapeInstance& instanceShape0, const ndShapeInstance& instanceShape1) const;
};

class ndMaterialHash
{
	public:
	ndMaterialHash()
		:m_key(0)
	{
	}

	ndMaterialHash(ndUnsigned32 low, ndUnsigned32 high)
		:m_lowKey(ndUnsigned32(ndMin(low, high)))
		,m_highKey(ndUnsigned32(ndMax(low, high)))
	{
	}

	bool operator<(const ndMaterialHash& other) const
	{
		return (m_key < other.m_key);
	}

	bool operator>(const ndMaterialHash& other) const
	{
		return (m_key > other.m_key);
	}

	union
	{
		ndUnsigned64 m_key;
		class
		{
			public:
			ndUnsigned32 m_lowKey;
			ndUnsigned32 m_highKey;
		};
	};
};

class ndMaterialGraph: public ndTree<ndApplicationMaterial*, ndMaterialHash, ndContainersFreeListAlloc<ndMaterialGraph*>>
{
	public:
	ndMaterialGraph();
	~ndMaterialGraph();
};

class ndContactCallback: public ndContactNotify
{
	public: 
	ndContactCallback();
	virtual ~ndContactCallback();
	virtual ndApplicationMaterial& RegisterMaterial(const ndApplicationMaterial& material, ndUnsigned32 id0, ndUnsigned32 id1);

	virtual void OnBodyAdded(ndBodyKinematic* const body) const;
	virtual void OnBodyRemoved(ndBodyKinematic* const body) const;

	virtual ndMaterial* GetMaterial(const ndContact* const contactJoint, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const;

	private:
	virtual bool OnAabbOverlap(const ndContact* const contactJoint, ndFloat32 timestep) const;
	virtual void OnContactCallback(const ndContact* const contactJoint, ndFloat32 timestep) const;
	virtual bool OnCompoundSubShapeOverlap(const ndContact* const contactJoint, ndFloat32 timestep, const ndShapeInstance* const instance0, const ndShapeInstance* const instance1) const;
	
	ndMaterialGraph m_materialGraph;
	ndApplicationMaterial m_defaultMaterial;
};

inline ndApplicationMaterial::ndApplicationMaterial()
	:ndMaterial()
{
}

inline ndApplicationMaterial::ndApplicationMaterial(const ndApplicationMaterial& copy)
	:ndMaterial(copy)
{
}

inline ndApplicationMaterial::~ndApplicationMaterial()
{
}

inline bool ndApplicationMaterial::OnAabbOverlap(const ndContact* const, ndFloat32, const ndShapeInstance&, const ndShapeInstance&) const
{
	return true;
}

inline void ndApplicationMaterial::OnContactCallback(const ndContact* const, ndFloat32) const
{
}

//**********************************************************************
// 
//**********************************************************************
inline ndMaterialGraph::ndMaterialGraph()
	:ndTree<ndApplicationMaterial*, ndMaterialHash, ndContainersFreeListAlloc<ndMaterialGraph*>>()
{
}

//**********************************************************************
// 
//**********************************************************************
inline ndContactCallback::ndContactCallback()
	:ndContactNotify(nullptr)
	,m_materialGraph()
	,m_defaultMaterial()
{
}

inline ndContactCallback::~ndContactCallback()
{
}

inline void ndContactCallback::OnBodyAdded(ndBodyKinematic* const) const
{
}

inline void ndContactCallback::OnBodyRemoved(ndBodyKinematic* const) const
{
}

inline ndMaterial* ndContactCallback::GetMaterial(const ndContact* const, const ndShapeInstance& instance0, const ndShapeInstance& instance1) const
{
	ndMaterialHash key(ndUnsigned32(instance0.GetMaterial().m_userId), ndUnsigned32(instance1.GetMaterial().m_userId));
	ndMaterialGraph::ndNode* const node = m_materialGraph.Find(key);
	return node ? node->GetInfo() : (ndMaterial*)&m_defaultMaterial;
}

#endif