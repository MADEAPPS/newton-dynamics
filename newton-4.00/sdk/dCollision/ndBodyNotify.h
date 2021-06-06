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

#ifndef __D_BODY_NOTIFY_H__
#define __D_BODY_NOTIFY_H__

#include "ndCollisionStdafx.h"

class ndBody;

D_MSV_NEWTON_ALIGN_32
class ndBodyNotify: public dClassAlloc
{
	public:  
	ndBodyNotify(const dVector& defualtGravity);
	D_COLLISION_API ndBodyNotify(const nd::TiXmlNode* const rootNode);
	virtual ~ndBodyNotify();

	ndBody* GetBody();
	const ndBody* GetBody() const;
	virtual void* GetUserData() const;
	dVector GetGravity() const;
	void SetGravity(const dVector& defualtGravity);

	virtual void OnTranform(dInt32 threadIndex, const dMatrix& matrix);

	D_COLLISION_API virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep);
	D_COLLISION_API virtual void Save(nd::TiXmlElement* const rootNode, const char* const assetPath) const;

	private:
	dVector m_defualtGravity;
	ndBody* m_body;
	friend class ndBody;

} D_GCC_NEWTON_ALIGN_32;

inline ndBodyNotify::ndBodyNotify(const dVector& defualtGravity)
	:dClassAlloc()
	,m_defualtGravity(defualtGravity & dVector::m_triplexMask)
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

inline dVector ndBodyNotify::GetGravity() const
{
	return m_defualtGravity;
}

inline void ndBodyNotify::SetGravity(const dVector& defualtGravity)
{
	m_defualtGravity = defualtGravity & dVector::m_triplexMask;
}


inline void* ndBodyNotify::GetUserData() const
{
	return nullptr;
}

inline void ndBodyNotify::OnTranform(dInt32, const dMatrix&)
{
}

#endif 

