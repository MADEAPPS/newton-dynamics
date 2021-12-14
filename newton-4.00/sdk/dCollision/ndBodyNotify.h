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

#ifndef __ND_BODY_NOTIFY_H__
#define __ND_BODY_NOTIFY_H__

#include "ndCollisionStdafx.h"

class ndBody;

D_MSV_NEWTON_ALIGN_32
class ndBodyNotify : public ndContainersFreeListAlloc<ndBodyNotify>
{
	public:  
	D_CLASS_REFLECTION(ndBodyNotify);
	ndBodyNotify(const ndVector& defualtGravity);
	D_COLLISION_API ndBodyNotify(const ndLoadSaveBase::ndLoadDescriptor& desc);
	virtual ~ndBodyNotify();

	ndBody* GetBody();
	const ndBody* GetBody() const;
	virtual void* GetUserData() const;
	ndVector GetGravity() const;
	void SetGravity(const ndVector& defualtGravity);

	virtual void OnTransform(ndInt32 threadIndex, const ndMatrix& matrix);

	D_COLLISION_API virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;
	D_COLLISION_API virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep);

	private:
	ndVector m_defualtGravity;
	ndBody* m_body;
	friend class ndBody;

} D_GCC_NEWTON_ALIGN_32;

inline ndBodyNotify::ndBodyNotify(const ndVector& defualtGravity)
	:ndContainersFreeListAlloc<ndBodyNotify>()
	,m_defualtGravity(defualtGravity & ndVector::m_triplexMask)
	,m_body(nullptr)
{
}

inline ndBodyNotify::~ndBodyNotify()
{
}

inline ndBody* ndBodyNotify::GetBody()
{
	return m_body;
}

inline const ndBody* ndBodyNotify::GetBody() const
{
	return m_body;
}

inline ndVector ndBodyNotify::GetGravity() const
{
	return m_defualtGravity;
}

inline void ndBodyNotify::SetGravity(const ndVector& defualtGravity)
{
	m_defualtGravity = defualtGravity & ndVector::m_triplexMask;
}


inline void* ndBodyNotify::GetUserData() const
{
	return nullptr;
}

inline void ndBodyNotify::OnTransform(ndInt32, const ndMatrix&)
{
}

#endif 

