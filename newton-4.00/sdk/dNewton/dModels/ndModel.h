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
class ndModelNotify;
class ndMultiBodyVehicle;
class ndModelArticulation;
class ndConstraintDebugCallback;

D_MSV_NEWTON_ALIGN_32
class ndModel: public ndContainersFreeListAlloc<ndModel>
{
	public:
	D_BASE_CLASS_REFLECTION(ndModel)

	D_NEWTON_API ndModel();
	D_NEWTON_API ndModel(const ndModel& src);

	D_NEWTON_API virtual ~ndModel ();
	D_NEWTON_API virtual ndModel* Clone() const;

	D_NEWTON_API virtual ndModel* GetAsModel();
	D_NEWTON_API virtual ndModelBase* GetAsModelBase();
	D_NEWTON_API virtual ndMultiBodyVehicle* GetAsMultiBodyVehicle();
	D_NEWTON_API virtual ndModelArticulation* GetAsModelArticulation();

	D_NEWTON_API ndSharedPtr<ndModelNotify>& GetNotifyCallback();
	D_NEWTON_API void SetNotifyCallback(const ndSharedPtr<ndModelNotify>& notifyCallback);

	protected:
	virtual void OnAddToWorld() {}
	virtual void OnRemoveFromToWorld() {}

	ndWorld* m_world;

	private:
	ndModelList::ndNode* m_worldNode;
	ndSpecialList<ndModel>::ndNode* m_deletedNode;
	ndSharedPtr<ndModelNotify> m_notifyCallback;
	D_MEMORY_ALIGN_FIXUP

	friend class ndWorld;
	friend class ndLoadSave;
	friend class ndModelList;
} D_GCC_NEWTON_ALIGN_32;

#endif 


