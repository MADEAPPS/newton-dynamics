/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_MODEL_H__
#define __ND_MODEL_H__

#include "ndNewtonStdafx.h"
#include "ndModelList.h"

class ndMultiBodyVehicle;
class ndConstraintDebugCallback;

template<class T>
class ndReferencedObjects : public ndList<ndSharedPtr<T>, ndContainersFreeListAlloc<ndSharedPtr<T>*>>
{
	public:
	ndReferencedObjects()
		:ndList<ndSharedPtr<T>, ndContainersFreeListAlloc<ndSharedPtr<T>*>>()
	{
	}

	void AddReferenceBody(ndSharedPtr<T>& object);
	ndSharedPtr<T>* FindReference(const T* const object) const;
};

D_MSV_NEWTON_ALIGN_32
class ndModel: public ndContainersFreeListAlloc<ndModel>
{
	public:
	D_CLASS_REFLECTION(ndModel);
	ndModel();
	D_NEWTON_API ndModel(const ndLoadSaveBase::ndLoadDescriptor& desc);
	virtual ~ndModel ();
	
	virtual void AddToWorld(ndWorld* const world);
	virtual void RemoveFromToWorld();

	virtual ndModel* GetAsModel();
	virtual ndMultiBodyVehicle* GetAsMultiBodyVehicle();
	virtual void Debug(ndConstraintDebugCallback& context) const;

	protected:
	D_NEWTON_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	virtual void Update(ndWorld* const world, ndFloat32 timestep);
	virtual void PostUpdate(ndWorld* const world, ndFloat32 timestep);
	virtual void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep);

	ndReferencedObjects<ndBodyKinematic> m_referencedBodies;
	ndReferencedObjects<ndJointBilateralConstraint> m_referencedJoints;
	ndModelList::ndNode* m_node;
	ndWorld* m_world;

	friend class ndWorld;
	friend class ndLoadSave;
	friend class ndModelList;
} D_GCC_NEWTON_ALIGN_32;

template<class T>
void ndReferencedObjects<T>::AddReferenceBody(ndSharedPtr<T>& object)
{
	T* const obj = *object;
	for (ndReferencedObjects<T>::ndNode* node = ndReferencedObjects<T>::GetFirst(); node; node = node->GetNext())
	{
		if (*node->GetInfo() == obj)
		{
			return;
		}
	}
	Append(object);
}

template<class T>
ndSharedPtr<T>* ndReferencedObjects<T>::FindReference(const T* const object) const
{
	for (ndReferencedObjects<T>::ndNode* node = GetFirst(); node; node = node->GetNext())
	{
		if (*node->GetInfo() == object)
		{
			return &node->GetInfo();
		}
	}
	return nullptr;
}

inline ndModel::ndModel()
	:ndContainersFreeListAlloc<ndModel>()
	,m_referencedBodies()
	,m_node(nullptr)
	,m_world(nullptr)
{
}

inline ndModel::~ndModel()
{
	ndAssert(!m_node);
}

inline ndModel* ndModel::GetAsModel()
{ 
	return this; 
}

inline ndMultiBodyVehicle* ndModel::GetAsMultiBodyVehicle()
{ 
	return nullptr; 
}

inline void ndModel::Debug(ndConstraintDebugCallback&) const
{
}

inline void ndModel::Update(ndWorld* const, ndFloat32)
{
}

inline void ndModel::PostUpdate(ndWorld* const, ndFloat32)
{
}

inline void ndModel::PostTransformUpdate(ndWorld* const, ndFloat32)
{
}

inline void ndModel::AddToWorld(ndWorld* const world)
{
	m_world = world;
}

inline void ndModel::RemoveFromToWorld()
{
}

#endif 


