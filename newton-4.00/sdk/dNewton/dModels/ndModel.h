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

class ndModelBase;
class ndMultiBodyVehicle;
class ndModelArticulation;
class ndConstraintDebugCallback;

D_MSV_NEWTON_ALIGN_32
class ndModel: public ndContainersFreeListAlloc<ndModel>
{
	public:
	D_BASE_CLASS_REFLECTION(ndModel)

	ndModel();
	virtual ~ndModel ();

	virtual ndModel* GetAsModel();
	virtual ndModelBase* GetAsModelBase();
	virtual ndMultiBodyVehicle* GetAsMultiBodyVehicle();
	virtual ndModelArticulation* GetAsModelArticulation();
	virtual void Debug(ndConstraintDebugCallback& context) const;

	protected:
	virtual void OnAddToWorld() = 0;
	virtual void OnRemoveFromToWorld() = 0;

	virtual void Update(ndWorld* const world, ndFloat32 timestep);
	virtual void PostUpdate(ndWorld* const world, ndFloat32 timestep);
	virtual void PostTransformUpdate(ndWorld* const world, ndFloat32 timestep);

	ndWorld* m_world;
	private:
	ndModelList::ndNode* m_worldNode;
	ndSpecialList<ndModel>::ndNode* m_deletedNode;

	friend class ndWorld;
	friend class ndLoadSave;
	friend class ndModelList;
} D_GCC_NEWTON_ALIGN_32;


inline ndModel::ndModel()
	:ndContainersFreeListAlloc<ndModel>()
	,m_world(nullptr)
	,m_worldNode(nullptr)
	,m_deletedNode(nullptr)
{
}

inline ndModel::~ndModel()
{
	ndAssert(!m_worldNode);
	ndAssert(!m_deletedNode);
}

inline ndModel* ndModel::GetAsModel()
{ 
	return this; 
}

inline ndModelBase* ndModel::GetAsModelBase()
{
	return nullptr;
}

inline ndMultiBodyVehicle* ndModel::GetAsMultiBodyVehicle()
{ 
	return nullptr; 
}

inline ndModelArticulation* ndModel::GetAsModelArticulation()
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

#endif 


