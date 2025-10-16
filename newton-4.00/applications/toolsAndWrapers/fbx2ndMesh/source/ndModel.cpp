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
#include "ndWorld.h"
#include "ndModel.h"
#include "ndModelNotify.h"

ndModel::ndModel()
	:ndContainersFreeListAlloc<ndModel>()
	,m_world(nullptr)
	,m_worldNode(nullptr)
	//,m_deletedNode(nullptr)
	,m_notifyCallback()
{
}

ndModel::ndModel(const ndModel& src)
	:ndContainersFreeListAlloc<ndModel>()
	,m_world(nullptr)
	,m_worldNode(nullptr)
	//,m_deletedNode(nullptr)
	,m_notifyCallback(*src.m_notifyCallback ? src.m_notifyCallback->Clone() : nullptr)
{
}

ndModel::~ndModel()
{
	//ndAssert(!m_worldNode);
	//ndAssert(!m_deletedNode);
}

ndModel* ndModel::Clone() const
{
	ndAssert(0);
	return nullptr;
}

ndWorld* ndModel::GetWorld() const
{
	return m_world;
}

ndModel* ndModel::GetAsModel()
{
	return this;
}

ndMultiBodyVehicle* ndModel::GetAsMultiBodyVehicle()
{
	return nullptr;
}

ndModelArticulation* ndModel::GetAsModelArticulation()
{
	return nullptr;
}

ndSharedPtr<ndModelNotify>& ndModel::GetNotifyCallback()
{
	return m_notifyCallback;
}

void ndModel::SetNotifyCallback(const ndSharedPtr<ndModelNotify>& notifyCallback)
{
	if (*m_notifyCallback)
	{
		m_notifyCallback->m_model = nullptr;
	}

	m_notifyCallback = notifyCallback;

	if (*m_notifyCallback)
	{
		m_notifyCallback->m_model = this;
	}
}