/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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
#include "ndBodyParticleSet.h"

ndBodyParticleSet::ndBodyParticleSet()
	:ndBody()
	,m_box0(ndFloat32(-1e10f))
	,m_box1(ndFloat32(1e10f))
	,m_gravity(ndVector::m_zero)
	,m_posit(1024)
	,m_veloc(1024)
	,m_listNode(nullptr)
{
}

ndBodyParticleSet::ndBodyParticleSet(const ndLoadSaveBase::ndLoadDescriptor& desc)
	:ndBody(desc)
	,m_box0(ndFloat32(-1e10f))
	,m_box1(ndFloat32(1e10f))
	,m_gravity(ndVector::m_zero)
	,m_posit(1024)
	,m_veloc(1024)
	,m_listNode(nullptr)
{
	//xmlNode->FirstChild("ndBodyKinematic"), shapesCache
	// nothing was saved
	dAssert(0);
}

ndBodyParticleSet::~ndBodyParticleSet()
{
}

//void ndBodyParticleSet::Save(const dLoadSaveBase::dSaveDescriptor& desc) const
void ndBodyParticleSet::Save(const ndLoadSaveBase::ndSaveDescriptor&) const
{
	dAssert(0);
	//nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBodyParticleSet", nodeid);
	//ndBody::Save(paramNode, assetPath, nodeid, shapesCache);
}