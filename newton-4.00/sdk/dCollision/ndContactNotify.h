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

#ifndef __ND_CONTACT_NOTIFY_H__
#define __ND_CONTACT_NOTIFY_H__

#include "ndCollisionStdafx.h"
#include "ndContactOptions.h"

class ndScene;
class ndContact;
class ndShapeInstance;

class ndMaterial
{
	public:
	ndMaterial()
	{
		m_restitution = dFloat32(0.4f);
		m_staticFriction0 = dFloat32(0.8f);
		m_staticFriction1 = dFloat32(0.8f);
		m_dynamicFriction0 = dFloat32(0.4f);
		m_dynamicFriction1 = dFloat32(0.4f);
		m_softness = dFloat32(0.1f);
		m_skinThickness = dFloat32 (0.0f);
		m_flags = m_collisionEnable | m_friction0Enable | m_friction1Enable;
		m_userFlags = 0;
	}

	dFloat32 m_restitution;
	dFloat32 m_staticFriction0;
	dFloat32 m_staticFriction1;
	dFloat32 m_dynamicFriction0;
	dFloat32 m_dynamicFriction1;
	dFloat32 m_softness;
	dFloat32 m_skinThickness;
	dUnsigned32 m_flags;
	dUnsigned32 m_userFlags;
};

D_MSV_NEWTON_ALIGN_32
class ndContactNotify: public ndClassAlloc
{
	public:
	ndContactNotify()
		:ndClassAlloc()
		,m_scene(nullptr)
	{
	}

	virtual ~ndContactNotify()
	{
	}

	virtual void OnBodyAdded(ndBodyKinematic* const) const
	{
	}

	virtual void OnBodyRemoved(ndBodyKinematic* const) const
	{
	}

	virtual ndMaterial GetMaterial(const ndContact* const, const ndShapeInstance&, const ndShapeInstance&) const
	{
		return ndMaterial();
	}

	//bool OnCompoundSubShapeOverlap(const ndContact* const contact, dFloat32 timestep, const ndShapeInstance* const subShapeA, const ndShapeInstance* const subShapeB);
	bool OnCompoundSubShapeOverlap(const ndContact* const, dFloat32, const ndShapeInstance* const, const ndShapeInstance* const)
	{
		return true;
	}

	//virtual bool OnAabbOverlap(const ndContact* const contact, dFloat32 timestep)
	virtual bool OnAabbOverlap(const ndContact* const, dFloat32)
	{
		return true;
	}

	virtual void OnContactCallback(dInt32, const ndContact* const, dFloat32)
	{
	}

	protected:
	ndScene* m_scene;
	friend class ndScene;
};

#endif
