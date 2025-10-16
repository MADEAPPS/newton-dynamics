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

#include "ndCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndModel.h"

ndModelList::ndModelList()
	:ndList<ndSharedPtr<ndModel>, ndContainersFreeListAlloc<ndSharedPtr<ndModel>*>>()
	,m_updateArray()
	,m_dirty(true)
{
}

ndArray<ndModel*>& ndModelList::GetUpdateList()
{
	return m_updateArray;
}

void ndModelList::UpdateDirtyList()
{
	if (m_dirty)
	{
		m_dirty = false;
		m_updateArray.SetCount(0);
		for (ndNode* node = GetFirst(); node; node = node->GetNext())
		{
			m_updateArray.PushBack(*node->GetInfo());
		}
	}
}

void ndModelList::AddModel(const ndSharedPtr<ndModel>& model, ndWorld* const world)
{
	ndAssert(!model->m_worldNode);
	if (!model->m_worldNode)
	{
		m_dirty = true;
		model->m_world = world;
		model->m_worldNode = Append(model);
		model->OnAddToWorld();
	}
}

void ndModelList::RemoveModel(ndModel* const model)
{
	ndNode* const node = model->m_worldNode;
	if (node)
	{
		m_dirty = true;
		model->OnRemoveFromToWorld();
		model->m_world = nullptr;
		model->m_worldNode = nullptr;
		Remove(node);
	}
}
