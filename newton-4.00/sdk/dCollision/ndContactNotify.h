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

#ifndef __ND_CONTACT_NOTIFY_H__
#define __ND_CONTACT_NOTIFY_H__

#include "ndCollisionStdafx.h"
#include "ndContactOptions.h"

class ndScene;
class ndContact;
class ndBodyKinematic;
class ndShapeInstance;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndMaterial : public ndContainersFreeListAlloc<ndMaterial>
{
	public:
	D_COLLISION_API ndMaterial();

	ndFloat32 m_restitution;
	ndFloat32 m_staticFriction0;
	ndFloat32 m_staticFriction1;
	ndFloat32 m_dynamicFriction0;
	ndFloat32 m_dynamicFriction1;
	ndFloat32 m_skinMargin;
	ndFloat32 m_softness;
	ndUnsigned32 m_flags;
	ndUnsigned32 m_userFlags;
} D_GCC_NEWTON_CLASS_ALIGN_32;

D_MSV_NEWTON_CLASS_ALIGN_32
class ndContactNotify: public ndClassAlloc
{
	public:
	D_COLLISION_API ndContactNotify(ndScene* const scene);
	D_COLLISION_API virtual ~ndContactNotify();
	D_COLLISION_API virtual ndMaterial* GetMaterial(const ndContact* const, const ndShapeInstance&, const ndShapeInstance&) const;

	protected:
	D_COLLISION_API virtual void OnBodyAdded(ndBodyKinematic* const) const;
	D_COLLISION_API virtual void OnBodyRemoved(ndBodyKinematic* const) const;
	D_COLLISION_API virtual void OnContactCallback(const ndContact* const, ndFloat32) const;
	D_COLLISION_API virtual bool OnAabbOverlap(const ndContact* const contact, ndFloat32 timestep) const;
	D_COLLISION_API virtual bool OnCompoundSubShapeOverlap(const ndContact* const contact, ndFloat32 timestep, const ndShapeInstance* const subShapeA, const ndShapeInstance* const subShapeB) const;
	
	ndMaterial m_default;

	private:
	ndScene* m_scene;

	friend class ndScene;
	friend class ndContactSolver;
	friend class ndPolygonMeshDesc;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif
