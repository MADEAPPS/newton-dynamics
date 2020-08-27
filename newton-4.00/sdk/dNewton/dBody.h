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
class dBroadPhaseBodyNode;

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

	virtual void OnApplyExternalForce(dInt32 threadIndex, dFloat32 timestep)
	{
	}

	virtual void OnTranform(dInt32 threadIndex, const dMatrix& matrix)
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

	D_NEWTON_API dInt32 GetId() const;
	D_NEWTON_API dNewton* GetNewton() const;

	D_NEWTON_API dVector GetOmega() const;
	D_NEWTON_API void SetOmega(const dVector& veloc);

	D_NEWTON_API dVector GetVelocity() const;
	D_NEWTON_API void SetVelocity(const dVector& veloc);

	D_NEWTON_API dMatrix GetMatrix() const;
	D_NEWTON_API void SetMatrix(const dMatrix& matrix);

	protected:
	dList<dBody*>::dListNode* GetNewtonNode() const;
	void SetNewtonNode(dNewton* const newton, dList<dBody*>::dListNode* const node);

	dBroadPhaseBodyNode* GetBroadPhaseNode() const;
	void SetBroadPhaseNode(dBroadPhaseBodyNode* const node);

	dMatrix m_matrix;
	dMatrix m_invWorldInertiaMatrix;
	dShapeInstance m_shapeInstance;
		
	dVector m_veloc;
	dVector m_omega;
	dVector m_localCentreOfMass;
	dVector m_globalCentreOfMass;
	dQuaternion m_rotation;
	dBodyNotify* m_notifyCallback;
	dNewton* m_newton;
	dBroadPhaseBodyNode* m_broadPhaseNode;
	dList<dBody*>::dListNode* m_newtonNode;

	dUnsigned32 m_uniqueID;
	static dUnsigned32 m_uniqueIDCount;
	friend class dNewton;
} D_GCC_VECTOR_ALIGNMENT;




#endif 

