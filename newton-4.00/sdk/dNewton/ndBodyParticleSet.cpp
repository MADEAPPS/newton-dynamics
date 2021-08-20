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

#include "dCoreStdafx.h"
#include "ndNewtonStdafx.h"
#include "ndWorld.h"
#include "ndBodyParticleSet.h"

ndBodyParticleSet::ndBodyParticleSet()
	:ndBody()
	,m_listNode(nullptr)
	//,m_accel(dVector::m_zero)
	//,m_alpha(dVector::m_zero)
	//,m_externalForce(dVector::m_zero)
	//,m_externalTorque(dVector::m_zero)
	//,m_impulseForce(dVector::m_zero)
	//,m_impulseTorque(dVector::m_zero)
	//,m_savedExternalForce(dVector::m_zero)
	//,m_savedExternalTorque(dVector::m_zero)
{
	//dAssert(0);
}

ndBodyParticleSet::ndBodyParticleSet(const dLoadSaveBase::dLoadDescriptor& desc)
	:ndBody(desc)
	//,m_accel(dVector::m_zero)
	//,m_alpha(dVector::m_zero)
	//,m_externalForce(dVector::m_zero)
	//,m_externalTorque(dVector::m_zero)
	//,m_impulseForce(dVector::m_zero)
	//,m_impulseTorque(dVector::m_zero)
	//,m_savedExternalForce(dVector::m_zero)
	//,m_savedExternalTorque(dVector::m_zero)
{
	//xmlNode->FirstChild("ndBodyKinematic"), shapesCache
	// nothing was saved
	dAssert(0);
}

ndBodyParticleSet::~ndBodyParticleSet()
{
}

void ndBodyParticleSet::Save(nd::TiXmlElement* const rootNode, const char* const assetPath, dInt32 shapeHash, dInt32 nodeHash) const
{
	dAssert(0);
	//nd::TiXmlElement* const paramNode = CreateRootElement(rootNode, "ndBodyParticleSet", nodeid);
	//ndBody::Save(paramNode, assetPath, nodeid, shapesCache);
}