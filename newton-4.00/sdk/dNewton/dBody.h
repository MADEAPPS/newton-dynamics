/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _D_BODY_H_
#define _D_BODY_H_

#include "dNewtonStdafx.h"
#include "dShapeInstance.h"

class dNewton;
class dDynamicBody;

D_MSC_VECTOR_ALIGNMENT
class dBodyNotify: public dClassAlloc
{
	public:  
	dBodyNotify()
		:dClassAlloc()
		,m_body(nullptr)
	{
	}

	virtual ~dBodyNotify()
	{
	}

	virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 tiemstep)
	{
	}

	protected:
	dBody* m_body;

	friend class dBody;

} D_GCC_VECTOR_ALIGNMENT;

D_MSC_VECTOR_ALIGNMENT
class dBody: public dClassAlloc
{
	public:
	D_NEWTON_API dBody();
	D_NEWTON_API virtual ~dBody();

	D_NEWTON_API virtual dBody* GetAsBody() { return this;}
	D_NEWTON_API virtual dDynamicBody* GetAsDynamicBody() { return nullptr; }

	D_NEWTON_API const dShapeInstance& GetCollisionShape() const;
	D_NEWTON_API void SetCollisionShape(const dShapeInstance& shapeInstance);

	D_NEWTON_API void SetCentreOfMass(const dVector& com);

	D_NEWTON_API void SetNotifyCallback(dBodyNotify* const notify);
	D_NEWTON_API dBodyNotify* GetNotifyCallback(dBodyNotify* const notify) const;

	dNewton* GetNewton() const;

	dVector GetOmega() const;
	void SetOmega(const dVector& veloc);

	dVector GetVelocity() const;
	void SetVelocity(const dVector& veloc);

	protected:
	dList<dBody*>::dListNode* GetNewtonNode() const;
	void SetNewtonNode(dNewton* const newton, dList<dBody*>::dListNode* const node);

	dMatrix m_matrix;
	dMatrix m_invWorldInertiaMatrix;
		
	dVector m_veloc;
	dVector m_omega;
	dVector m_localCentreOfMass;
	dVector m_globalCentreOfMass;

	dQuaternion m_rotation;

	dShapeInstance m_shapeInstance;
	dBodyNotify* m_notifyCallback;
	dNewton* m_newton;
	dList<dBody*>::dListNode* m_newtonNode;
	friend class dNewton;
} D_GCC_VECTOR_ALIGNMENT;


inline dNewton* dBody::GetNewton() const
{
	return m_newton;
}

inline void dBody::SetNewtonNode(dNewton* const newton, dList<dBody*>::dListNode* const node)
{
	m_newton = newton;
	m_newtonNode = node;
}

inline dList<dBody*>::dListNode* dBody::GetNewtonNode() const
{
	return m_newtonNode;
}

inline dVector dBody::GetOmega() const
{
	return m_omega;
}

inline void dBody::SetOmega(const dVector& veloc)
{
	m_omega = veloc;
}

inline dVector dBody::GetVelocity() const
{
	return m_veloc;
}

inline void dBody::SetVelocity(const dVector& veloc)
{
	m_veloc = veloc;
}


#endif 

