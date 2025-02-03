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
#include "ndCollisionStdafx.h"
#include "ndContactNotify.h"

ndMaterial::ndMaterial()
	:ndContainersFreeListAlloc<ndMaterial>()
{
	m_restitution = ndFloat32(0.4f);
	m_staticFriction0 = ndFloat32(0.8f);
	m_staticFriction1 = ndFloat32(0.8f);
	m_dynamicFriction0 = ndFloat32(0.4f);
	m_dynamicFriction1 = ndFloat32(0.4f);
	m_skinMargin = ndFloat32(0.0f);
	m_softness = ndFloat32(0.1f);
	m_flags = m_collisionEnable | m_friction0Enable | m_friction1Enable;
	m_userFlags = 0;
}

ndContactNotify::ndContactNotify(ndScene* const scene)
	:ndClassAlloc()
	, m_scene(scene)
{
}

ndContactNotify::~ndContactNotify()
{
}

ndMaterial* ndContactNotify::GetMaterial(const ndContact* const, const ndShapeInstance&, const ndShapeInstance&) const
{
	return (ndMaterial*)&m_default;
}

void ndContactNotify::OnBodyAdded(ndBodyKinematic* const) const
{
}

void ndContactNotify::OnBodyRemoved(ndBodyKinematic* const) const
{
}

//bool OnCompoundSubShapeOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance* const subShapeA, const ndShapeInstance* const subShapeB);
bool ndContactNotify::OnCompoundSubShapeOverlap(const ndContact* const, ndFloat32, const ndShapeInstance* const, const ndShapeInstance* const) const
{
	return true;
}

//virtual bool OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep)
bool ndContactNotify::OnAabbOverlap(const ndContact* const, ndFloat32) const
{
	return true;
}

void ndContactNotify::OnContactCallback(const ndContact* const, ndFloat32) const
{
}
