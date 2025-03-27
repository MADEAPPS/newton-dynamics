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
#include "ndModelNotify.h"

ndModelNotify::ndModelNotify(const ndModelNotify& src)
	:ndContainersFreeListAlloc<ndModelNotify>()
	,m_model(src.m_model)
{
}

ndModelNotify::ndModelNotify()
	:ndContainersFreeListAlloc<ndModelNotify>()
	,m_model(nullptr)
{
}

ndModelNotify* ndModelNotify::Clone() const
{
	ndAssert(0);
	return new ndModelNotify();
}

ndModelNotify::~ndModelNotify()
{
}

ndModel* ndModelNotify::GetModel() const
{
	return m_model;
}

void ndModelNotify::SetModel(ndModel* const model)
{
	m_model = model;
}

void ndModelNotify::Debug(ndConstraintDebugCallback&) const
{
}

void ndModelNotify::Update(ndFloat32)
{
}

void ndModelNotify::PostUpdate(ndFloat32)
{
}

void ndModelNotify::PostTransformUpdate(ndFloat32)
{
}
